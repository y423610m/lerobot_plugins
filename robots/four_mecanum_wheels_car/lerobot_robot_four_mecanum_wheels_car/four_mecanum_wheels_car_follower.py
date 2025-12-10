from __future__ import annotations

from typing import Any, Optional, Tuple
import threading
import asyncio
import inspect
import numpy as np
from functools import cached_property

from lerobot.cameras import make_cameras_from_configs
from lerobot.utils.errors import DeviceNotConnectedError, DeviceAlreadyConnectedError
from lerobot.robots.robot import Robot

from .config_four_mecanum_wheels_car_follower import FourMecanumWheelsCarFollowerConfig


from gpiozero import Servo, DigitalOutputDevice

class Wheel:
    def __init__(self, pin_pwm, pin_forward, pin_backward):
        self.pin_pwm = pin_pwm
        self.pin_forward = pin_forward 
        self.pin_backward = pin_backward
        print(f"{self.pin_pwm=} {self.pin_forward=} {self.pin_backward=}")

    def connect(self):
        # gpiozero implicitly depends in lgpio for factory. Make sure lgpio is installed.
        self.servo_pwm =  Servo(self.pin_pwm, frame_width=20/1000, min_pulse_width=0.1*20/1000, max_pulse_width=0.99*20/1000)    
        self.servo_forward =  DigitalOutputDevice(self.pin_forward)
        self.servo_backward =  DigitalOutputDevice(self.pin_backward)

    def set_speed(self, speed):
        speed = max(-1.0, min(1.0, speed))
        self.servo_pwm.value = abs(speed)
        if speed > 0.0:
            self.servo_forward.on()
            self.servo_backward.off()
        elif speed < 0.0:
            self.servo_forward.off()
            self.servo_backward.on()
        else:
            self.servo_forward.off()
            self.servo_backward.off()

    def move_forward(self, speed=0.5):
        self.servo_pwm.value = speed
        self.servo_forward.on()
        self.servo_backward.off()
        print("f")

    def move_backward(self, speed=0.5):
        self.servo_pwm.value = speed
        self.servo_forward.off()
        self.servo_backward.on()
        print("b")

    def stop(self):
        self.servo_forward.off()
        self.servo_backward.off()

class FourMecanumWheelsCarFollower(Robot):

    config_class = FourMecanumWheelsCarFollowerConfig
    name = "four_mecanum_wheels_car_follower"

    def __init__(self, config: FourMecanumWheelsCarFollowerConfig):
        super().__init__(config)
        self.config = config

        print(f"{config=}")

        self.cameras = make_cameras_from_configs(config.cameras)

        self.wheels = {}
        if self.config.rf:
            self.wheels["rf"] = Wheel(self.config.rf.pwm, self.config.rf.forward, self.config.rf.backward)
        if self.config.rb:
            self.wheels["rb"] = Wheel(self.config.rb.pwm, self.config.rb.forward, self.config.rb.backward)
        if self.config.lf:
            self.wheels["lf"] = Wheel(self.config.lf.pwm, self.config.lf.forward, self.config.lf.backward)
        if self.config.rb:
            self.wheels["lb"] = Wheel(self.config.lb.pwm, self.config.lb.forward, self.config.lb.backward)

        self._is_connected = False
        self._is_calibrated = True

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        for wheel_key, wheel in self.wheels.items():
            self.wheels[wheel_key].connect()

        for cam in self.cameras.values():
            cam.connect()

        self._is_connected = True
        print(f"{self} connected.")

    def disconnect(self) -> None:
        if not self.is_connected:
            return
        for cam in self.cameras.values():
            cam.disconnect()
        self._is_connected = False
        print(f"{self} disconnected.")

    def calibrate(self) -> None:
        return

    def configure(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @is_connected.setter
    def is_connected(self, value: bool) -> None:
        self._is_connected = value

    @property
    def is_calibrated(self) -> bool:
        return self.is_connected

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return self._cam_shape

    @cached_property
    def _state_ft(self) -> dict[str, type]:
        return dict.fromkeys(
            (
                "x.vel",
                "y.vel",
                "theta.vel",
            ),
            float,
        )

    @property
    def observation_features(self) -> dict[str, Any]:
        return {**self._state_ft, **self._cameras_ft}

    @property
    def action_features(self) -> dict[str, type]:
        return self._state_ft

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        wheeel_speeeds = {
            "rf": 0.0,
            "lf": 0.0,
            "rb": 0.0,
            "lb": 0.0,
        }
        if action["x.vel"] > 0.0:
            wheeel_speeeds["rf"] += 1.0
            wheeel_speeeds["lf"] += 1.0
            wheeel_speeeds["rb"] += 1.0
            wheeel_speeeds["lb"] += 1.0
        elif action["x.vel"] < 0.0:
            wheeel_speeeds["rf"] -= 1.0
            wheeel_speeeds["lf"] -= 1.0
            wheeel_speeeds["rb"] -= 1.0
            wheeel_speeeds["lb"] -= 1.0

        if action["y.vel"] > 0.0:
            wheeel_speeeds["rf"] += 0.5
            wheeel_speeeds["lf"] += 0.5
            wheeel_speeeds["rb"] -= 0.5
            wheeel_speeeds["lb"] -= 0.5
        elif action["y.vel"] < 0.0:
            wheeel_speeeds["rf"] -= 0.5
            wheeel_speeeds["lf"] -= 0.5
            wheeel_speeeds["rb"] += 0.5
            wheeel_speeeds["lb"] += 0.5

        if action["theta.vel"] > 0.0:
            wheeel_speeeds["rf"] += 0.3
            wheeel_speeeds["lf"] -= 0.3
            wheeel_speeeds["rb"] += 0.3
            wheeel_speeeds["lb"] -= 0.3
        elif action["theta.vel"] < 0.0:
            wheeel_speeeds["rf"] -= 0.3
            wheeel_speeeds["lf"] += 0.3
            wheeel_speeeds["rb"] -= 0.3
            wheeel_speeeds["lb"] += 0.3

        for wheel_key, speed in wheeel_speeeds.items():
            if wheel_key in self.wheels:
                self.wheels[wheel_key].set_speed(speed)
        return action

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        obs: dict[str, Any] = {
            "x.val": float(0.0),
            "y.val": float(0.0),
            "theta.val": float(0.0),
        }

        for cam_key, cam in self.cameras.items():
            h, w, c = self._cam_shape[cam_key]
            frame = None
            try:
                if hasattr(cam, "async_read"):
                    frame = cam.async_read()
                elif hasattr(cam, "read"):
                    ok, frame = cam.read()
                    if ok is False:
                        frame = None
            except Exception:
                frame = None

            if frame is None or not isinstance(frame, np.ndarray):
                frame = np.zeros((h, w, c), dtype=np.uint8)

            obs[cam_key] = frame

        return obs