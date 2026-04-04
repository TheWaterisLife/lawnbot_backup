#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32

import board
import busio
import smbus2
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# MCP23017 registers (BANK=0 default)
_IODIRB = 0x01
_OLATB  = 0x15

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        
        # Parameters
        self.declare_parameter('voltage_divider_r1', 150000.0)
        self.declare_parameter('voltage_divider_r2', 22000.0)
        self.declare_parameter('min_voltage', 12.0)
        self.declare_parameter('max_voltage', 13.0)
        self.declare_parameter('ads1015_address', 0x49)
        self.declare_parameter('gpio_extender_address', 0x27)
        self.declare_parameter('adc_gain', 1)
        self.declare_parameter('adc_channel', 3)
        self.declare_parameter('publish_rate', 1.0)
        
        # Get params
        self.r1 = self.get_parameter('voltage_divider_r1').value
        self.r2 = self.get_parameter('voltage_divider_r2').value
        self.min_voltage = self.get_parameter('min_voltage').value
        self.max_voltage = self.get_parameter('max_voltage').value
        ads_addr = self.get_parameter('ads1015_address').value
        self.mcp_addr = self.get_parameter('gpio_extender_address').value
        self.gain = self.get_parameter('adc_gain').value
        self.adc_channel = self.get_parameter('adc_channel').value
        rate = self.get_parameter('publish_rate').value
        
        # Publishers
        self.battery_pub = self.create_publisher(BatteryState, '/battery', 10)
        self.voltage_pub = self.create_publisher(Float32, '/battery/voltage', 10)
        
        self.get_logger().info(f"Connecting to I2C (ADS: {hex(ads_addr)} ch:{self.adc_channel}, MCP: {hex(self.mcp_addr)})")
        
        # I2C Setup
        try:
            i2c = board.I2C()  # Uses board.SCL and board.SDA
        except Exception as e:
            self.get_logger().error(f"Failed to open I2C bus: {e}")
            self.ads = None
            self.smb = None
            self.timer = self.create_timer(1.0 / rate, self.timer_callback)
            return

        # ADS1015 Setup (voltage reading)
        self.ads = None
        try:
            self.ads = ADS.ADS1015(i2c, address=ads_addr)
            self.ads.gain = self.gain
            self.chan = AnalogIn(self.ads, self.adc_channel)
            self.get_logger().info("ADS1015 (ADC) connected successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to ADS1015: {e}")
            self.ads = None

        # MCP23017 Setup via smbus2 (direct register access, no glitching)
        self.smb = None
        try:
            self.smb = smbus2.SMBus(1)
            iodirb = self.smb.read_byte_data(self.mcp_addr, _IODIRB)
            self.smb.write_byte_data(self.mcp_addr, _IODIRB, iodirb & 0xE0)  # GPB0-4 as output
            self.smb.write_byte_data(self.mcp_addr, _OLATB, 0x1F)            # all OFF (HIGH)
            self.get_logger().info("MCP23017 (LEDs) connected via smbus2")
        except Exception as e:
            self.get_logger().warn(f"MCP23017 not found (LEDs disabled): {e}")
            self.smb = None

        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

    def timer_callback(self):
        if self.ads is None:
            return

        try:
            # Read Raw Voltage (at ADC pin)
            raw_voltage = self.chan.voltage
            
            # Calculate Battery Voltage based on divider
            # Vout = Vin * (R2 / (R1 + R2))
            # Vin = Vout * ((R1 + R2) / R2)
            divider_ratio = (self.r1 + self.r2) / self.r2
            battery_voltage = raw_voltage * divider_ratio
            
            # Calculate Percentage
            percentage = self.voltage_to_percentage(battery_voltage)
            
            # Update LEDs
            self.update_leds(percentage)
            
            # Publish BatteryState
            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.voltage = battery_voltage
            msg.percentage = percentage / 100.0
            msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
            msg.present = True
            msg.design_capacity = 30.0 # Ah
            
            # Simple health check
            if battery_voltage < self.min_voltage:
                 msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
            elif battery_voltage > 15.0:
                 msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE
            else:
                 msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

            # Simple status check (logic could be improved with current sensor)
            if battery_voltage > self.max_voltage:
                 msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
            else:
                 msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            
            self.battery_pub.publish(msg)
            
            # Publish float voltage
            v_msg = Float32()
            v_msg.data = battery_voltage
            self.voltage_pub.publish(v_msg)
            
        except Exception as e:
            self.get_logger().warn(f"Error reading battery: {e}")

    def voltage_to_percentage(self, voltage):
        # LiFePO4 Discharge Curve (13.0V full → 12.0V cutoff)
        discharge_curve = [
            (13.0, 100.0),
            (12.9, 90.0),
            (12.8, 80.0),  # Nominal
            (12.7, 70.0),
            (12.6, 60.0),
            (12.5, 50.0),
            (12.4, 40.0),
            (12.3, 30.0),
            (12.2, 20.0),  # Knee
            (12.1, 10.0),
            (12.0, 0.0),   # Cutoff
        ]
        
        # Clamp
        if voltage >= discharge_curve[0][0]:
            return 100.0
        if voltage <= discharge_curve[-1][0]:
            return 0.0
            
        # Linear Interpolation
        for i in range(len(discharge_curve) - 1):
            upper_v, upper_p = discharge_curve[i]
            lower_v, lower_p = discharge_curve[i+1]
            
            if lower_v <= voltage <= upper_v:
                # Interpolate
                ratio = (voltage - lower_v) / (upper_v - lower_v)
                return lower_p + ratio * (upper_p - lower_p)
                
        return 0.0

    def update_leds(self, percentage):
        if self.smb is None:
            return

        # Battery bar LEDs on GPB0-GPB4 (active-low: 0=ON, 1=OFF)
        #   bit 0 GPB0 → GREEN  : 80%+   bit 1 GPB1 → GREEN : 60%+ (HW broken)
        #   bit 2 GPB2 → AMBER  : 40%+   bit 3 GPB3 → AMBER : 20%+
        #   bit 4 GPB4 → RED    :  0%+
        thresholds = [80, 60, 40, 20, 0]

        try:
            olatb = self.smb.read_byte_data(self.mcp_addr, _OLATB)
            olatb |= 0x1F  # start with all 5 bits HIGH (OFF)
            for i, thr in enumerate(thresholds):
                if percentage >= thr:
                    olatb &= ~(1 << i)  # pull LOW = ON
            self.smb.write_byte_data(self.mcp_addr, _OLATB, olatb)
        except Exception as e:
            self.get_logger().warn(f"Error updating LEDs: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
