#!/usr/bin/env python3
"""
LED Pin Mapping Discovery Script
Press Enter to move to the next pin. Type what you see (e.g. "green", "amber", "red", "none").
"""
import board
import busio
import digitalio
from adafruit_mcp230xx.mcp23017 import MCP23017

MCP_ADDRESS = 0x27

def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    mcp = MCP23017(i2c, address=MCP_ADDRESS)
    print(f"MCP23017 connected at {hex(MCP_ADDRESS)}\n")

    # Initialize all 16 pins as outputs, all OFF
    pins = []
    for i in range(16):
        pin = mcp.get_pin(i)
        pin.direction = digitalio.Direction.OUTPUT
        pin.value = False
        pins.append(pin)

    # Step 1: Turn ALL pins ON
    print("=" * 50)
    print("STEP 1: Turning ALL 16 pins ON")
    print("=" * 50)
    for p in pins:
        p.value = True
    input("How many LEDs are on? Press Enter to continue...")

    # Turn all OFF
    for p in pins:
        p.value = False

    # Step 2: Test each pin one by one
    print("\n" + "=" * 50)
    print("STEP 2: Testing each pin one at a time")
    print("=" * 50)

    results = {}
    for i in range(16):
        bank = "GPA" if i < 8 else "GPB"
        bank_pin = i if i < 8 else i - 8
        
        pins[i].value = True
        answer = input(f"Pin {i:2d} ({bank}{bank_pin}) is ON — What LED? (green/amber/red/none): ").strip().lower()
        results[i] = answer
        pins[i].value = False

    # Summary
    print("\n" + "=" * 50)
    print("RESULTS SUMMARY")
    print("=" * 50)
    for i, led in results.items():
        if led and led != "none" and led != "n":
            bank = "GPA" if i < 8 else "GPB"
            bank_pin = i if i < 8 else i - 8
            print(f"  Pin {i:2d} ({bank}{bank_pin}) -> {led}")

    # Turn all OFF
    for p in pins:
        p.value = False

if __name__ == "__main__":
    main()
