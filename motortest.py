from gpiozero import PWMOutputDevice, DigitalOutputDevice
from time import sleep

# ENABLE PINS
R_EN = DigitalOutputDevice(23)
L_EN = DigitalOutputDevice(24)

# PWM PINS
R_PWM = PWMOutputDevice(12, frequency=1000)
L_PWM = PWMOutputDevice(19, frequency=1000)

# SPEED SETTINGS
low_speed = 0.2     # 20%
high_speed = 0.7    # 70%
interval = 3        # seconds

print("Motor PWM Toggle Started")

# Enable driver
R_EN.on()
L_EN.on()

try:
    while True:

        print("Speed: 20%")
        L_PWM.value = 0
        R_PWM.value = low_speed
        sleep(interval)

        print("Speed: 70%")
        L_PWM.value = 0
        R_PWM.value = high_speed
        sleep(interval)

except KeyboardInterrupt:

    print("\nStopping motor")

    # Stop motor
    R_PWM.value = 0
    L_PWM.value = 0

    # Disable driver
    R_EN.off()
    L_EN.off()

