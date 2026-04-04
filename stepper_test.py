from gpiozero import OutputDevice
from time import sleep

# IN1..IN4 -> GPIO17,18,27,22
pins = [OutputDevice(17), OutputDevice(18), OutputDevice(27), OutputDevice(22)]

# Half-step sequence for 28BYJ-48
seq = [
    (1,0,0,0),
    (1,1,0,0),
    (0,1,0,0),
    (0,1,1,0),
    (0,0,1,0),
    (0,0,1,1),
    (0,0,0,1),
    (1,0,0,1),
]

def run(delay=0.003):
    for s in seq:
        for p, v in zip(pins, s):
            p.value = v
        sleep(delay)

print("Running stepper. Ctrl+C to stop.")
try:
    while True:
        run()
finally:
    for p in pins:
        p.off()

