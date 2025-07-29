#!/usr/bin/env python
# -*- coding: utf-8 -*-


class VehicleStatus(object):
    __slots__ = (
        # general vehicle status
        "speed",
        "throttle",
        "brake",
        "steering",
        "turn_light",
        "driving_mode",
        "gear",
        "parking_brake",
        "battery",

        # LMT status
        "motor1_current",
        "motor1_speed",
        "motor1_work_mode",
        "motor1_remote_status",
        "motor1_temperature",
        "motor1_pulse_count",
        "motor2_current",
        "motor2_speed",
        "motor2_work_mode",
        "motor2_remote_status",
        "motor2_temperature",
        "motor2_pulse_count",

        "timestamp",
    )

    def __init__(self):
        self.speed = 0.0        # 车速 (km/h)
        self.throttle = 0.0     # 油门百分比 (%)
        self.steering = 0.0     # (%)
        self.brake = 0.0        # 制动百分比 (%)
        self.turn_light = "None"  # 转向灯状态 ("None", "Left", "Right", "Hazard")
        self.driving_mode = "MANUAL"  # 驾驶模式 ("MANUAL", "AUTO")
        self.gear = "P"  # 档位 ("P", "R", "N", "D")
        self.parking_brake = "Off"  # 驻车制动状态 ("Off", "On")
        self.battery = 100

        # vehicle LMT
        self.motor1_current = 0.0
        self.motor1_speed = 0.0
        self.motor1_work_mode = 0
        self.motor1_remote_status = 0
        self.motor1_temperature = 0.0
        self.motor1_pulse_count = 0
        self.motor2_current = 0.0
        self.motor2_speed = 0.0
        self.motor2_work_mode = 0
        self.motor2_remote_status = 0
        self.motor2_temperature = 0.0
        self.motor2_pulse_count = 0

        self.timestamp = None  # 时间戳
