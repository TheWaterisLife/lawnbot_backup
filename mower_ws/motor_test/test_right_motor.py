from gpiozero import PWMOutputDevice, DigitalOutputDevice
from time import sleep

# ENABLE PINS
R_EN = DigitalOutputDevice(23)   # Pin 16
L_EN = DigitalOutputDevice(24)   # Pin 18

# PWM PINS (changed)
R_PWM = PWMOutputDevice(12, frequency=1000)   # Pin 32
L_PWM = PWMOutputDevice(19, frequency=1000)   # Pin 35

# SETTINGS
speed = 0.6
pulse_time = 0.5
interval = 3.0

print("3 Second Pulse Test Started")

# Enable driver
R_EN.on()
L_EN.on()

try:
    while True:
        print("PULSE ON")
        L_PWM.value = 0
        R_PWM.value = speed
        sleep(pulse_time)

        print("PULSE OFF")
        R_PWM.value = 0
        L_PWM.value = 0
        sleep(interval - pulse_time)

except KeyboardInterrupt:
    print("\nStopping motor")
    R_PWM.value = 0
    L_PWM.value = 0
    R_EN.off()
    L_EN.off()

