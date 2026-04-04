# mower_ws/src/lawnbot_motors/motor_hw.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict
from gpiozero import PWMOutputDevice, DigitalOutputDevice


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


@dataclass
class MotorPins:
    r_en: int
    l_en: int
    r_pwm: int
    l_pwm: int


class BTS7960Motor:
    """
    speed in [-1.0, +1.0]
      forward:  R_PWM = duty, L_PWM = 0
      reverse:  L_PWM = duty, R_PWM = 0
    """
    def __init__(
        self,
        name: str,
        pins: MotorPins,
        pwm_frequency_hz: int = 2000,
        max_duty: float = 1.0,
        invert: bool = False,
    ):
        self.name = name
        self.pins = pins
        self.max_duty = _clamp(float(max_duty), 0.0, 1.0)
        self.invert = bool(invert)

        self.r_en = DigitalOutputDevice(pins.r_en)
        self.l_en = DigitalOutputDevice(pins.l_en)
        self.r_pwm = PWMOutputDevice(pins.r_pwm, frequency=pwm_frequency_hz)
        self.l_pwm = PWMOutputDevice(pins.l_pwm, frequency=pwm_frequency_hz)

        self.enable()
        self.stop()

    def enable(self) -> None:
        self.r_en.on()
        self.l_en.on()

    def disable(self) -> None:
        self.stop()
        self.r_en.off()
        self.l_en.off()

    def stop(self) -> None:
        self.r_pwm.value = 0.0
        self.l_pwm.value = 0.0

    def set_speed(self, speed: float) -> None:
        speed = float(speed)
        if self.invert:
            speed = -speed

        speed = _clamp(speed, -1.0, 1.0)
        duty = _clamp(abs(speed) * self.max_duty, 0.0, self.max_duty)

        if speed > 0:
            self.l_pwm.value = 0.0
            self.r_pwm.value = duty
        elif speed < 0:
            self.r_pwm.value = 0.0
            self.l_pwm.value = duty
        else:
            self.stop()


def build_motors_from_config(cfg: dict) -> Dict[str, BTS7960Motor]:
    pwm_freq = int(cfg.get("gpio", {}).get("pwm_frequency_hz", 1000))
    motors_cfg = cfg.get("motors", {})

    motors: Dict[str, BTS7960Motor] = {}
    for name, m in motors_cfg.items():
        pins = MotorPins(
            r_en=int(m["r_en"]),
            l_en=int(m["l_en"]),
            r_pwm=int(m["r_pwm"]),
            l_pwm=int(m["l_pwm"]),
        )

        motors[name] = BTS7960Motor(
            name=name,
            pins=pins,
            pwm_frequency_hz=pwm_freq,
            max_duty=float(m.get("max_duty", 1.0)),
            invert=bool(m.get("invert", False)),
        )

    return motors

