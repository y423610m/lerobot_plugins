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


from gpiozero import Servo, LED, DigitalOutputDevice

class Wheel:
    def __init__(self, pin_pwm, pin_forward, pin_backward):
        self.pin_pwm = pin_pwm
        self.pin_forward = pin_forward 
        self.pin_backward = pin_backward
        print(f"{self.pin_pwm=} {self.pin_forward=} {self.pin_backward=}")

    def connect(self):
        self.servo_pwm =  Servo(self.pin_pwm, frame_width=20/1000, min_pulse_width=0.1*20/1000, max_pulse_width=0.99*20/1000)    
        self.servo_forward =  DigitalOutputDevice(self.pin_forward)
        self.servo_backward =  DigitalOutputDevice(self.pin_backward)

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

        self.cameras = make_cameras_from_configs(config.cameras)

        self.wheels = {}
        for wheel_key, motor_settings in self.config.channles_settings.items():
            self.wheels[wheel_key] = Wheel(motor_settings['pwm'], motor_settings['forward'], motor_settings['backward'])

        self._is_connected = False
        self._is_calibrated = True


        self._position = 0.0


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

        # TODO handle wheel control
        print(f"sent {action=}")
        if action['x.vel'] > 0.0:
            for wheel_key, wheel in self.wheels.items():
                wheel.move_forward()
        elif action['x.vel'] < 0.0:
            for wheel_key, wheel in self.wheels.items():
                wheel.move_backward()
        else:
            for wheel_key, wheel in self.wheels.items():
                wheel.stop()
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