from gpiozero import DigitalOutputDevice, PWMOutputDevice
from time import sleep

# =========================
# RIGHT MOTOR
# =========================
R_R_EN = DigitalOutputDevice(5)
R_L_EN = DigitalOutputDevice(6)

R_R_PWM = PWMOutputDevice(19, frequency=1000)
R_L_PWM = PWMOutputDevice(12, frequency=1000)

# =========================
# LEFT MOTOR
# =========================
L_R_EN = DigitalOutputDevice(23)
L_L_EN = DigitalOutputDevice(24)

L_R_PWM = PWMOutputDevice(18, frequency=1000)
L_L_PWM = PWMOutputDevice(13, frequency=1000)

# =========================
# BLADE MOTOR
# =========================
B_R_EN = DigitalOutputDevice(20)
B_L_EN = DigitalOutputDevice(16)

B_R_PWM = PWMOutputDevice(26, frequency=1000)
B_L_PWM = PWMOutputDevice(21, frequency=1000)

# =========================
# SETTINGS
# =========================
speed = 0.6
pulse_time = 1.0
interval = 3.0

print("=== MOTOR TEST STARTED ===")

# ENABLE ALL DRIVERS
R_R_EN.on(); R_L_EN.on()
L_R_EN.on(); L_L_EN.on()
B_R_EN.on(); B_L_EN.on()

try:
    # ---------- RIGHT ----------
    print("\nRIGHT MOTOR")
    R_R_PWM.value = speed
    sleep(pulse_time)
    R_R_PWM.value = 0
    sleep(interval)

    # ---------- LEFT ----------
    print("\nLEFT MOTOR")
    L_R_PWM.value = speed
    sleep(pulse_time)
    L_R_PWM.value = 0
    sleep(interval)

    # ---------- BLADE ----------
    print("\nBLADE MOTOR")
    B_R_PWM.value = speed
    sleep(pulse_time)
    B_R_PWM.value = 0
    sleep(interval)

    print("\nTEST COMPLETE")

except KeyboardInterrupt:
    print("\nSTOPPED BY USER")

finally:
    # FULL STOP SAFETY
    R_R_PWM.value = 0
    R_L_PWM.value = 0
    L_R_PWM.value = 0
    L_L_PWM.value = 0
    B_R_PWM.value = 0
    B_L_PWM.value = 0

    R_R_EN.off(); R_L_EN.off()
    L_R_EN.off(); L_L_EN.off()
    B_R_EN.off(); B_L_EN.off()

