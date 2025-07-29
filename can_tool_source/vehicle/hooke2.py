#!/usr/bin/env python
# -*- coding: utf-8 -*-

import struct
import can
import time
from vehicle.vehicle_status import VehicleStatus

HOOKE2_SEND_PERIOD = 0.02  # 发送周期，单位：秒

# 创建CAN解析类
class HOOKE2CanReportHandler:
    def __init__(self, logger):
        self.log = logger
        self.vehicle_status = VehicleStatus()
        # 定义目标 ID 和对应的处理函数
        self.dtv_can_report_ids = {
            0x500: self.on_throttle_report_500,
            0x501: self.on_brake_report_501,
            0x502: self.on_steering_report_502,
            0x503: self.on_gear_report_503,
            0x504: self.on_park_report_504,
            0x505: self.on_vcu_report_505,
            0x506: self.on_wheel_speed_report_506,
            0x507: self.on_ultr_sensor_507,
            0x508: self.on_ultr_sensor_508,
            0x509: self.on_ultr_sensor_509,
            0x510: self.on_ultr_sensor_510,
            0x511: self.on_ultr_sensor_511,
            0x512: self.on_bms_report_512,
            0x514: self.on_vin_report_514,
            0x515: self.on_vin_report_515,
            0x516: self.on_vin_report_516,
        }

    def handle_message(self, msg: can.Message):
        """
        处理 CAN 消息。
        """
        if msg is not None:
            if msg.arbitration_id in self.dtv_can_report_ids:
                self.dtv_can_report_ids[msg.arbitration_id](msg.data)
                
    def on_throttle_report_500(self, data):
        """
        处理 Throttle Report (0x500) 消息。
        """
        throttle_pedal_actual = (data[3] << 8 | data[4]) * 0.1
        throttle_flt2_type = data[2]
        throttle_flt1_type = data[1]
        # 0: 'MANUAL', 1: 'AUTO', 2: 'TAKEOVER', 3: 'STANDBY'
        throttle_en_state = data[0] & 0x03

        self.log.debug(
            f"Throttle Report (0x500): "
            f"Pedal: {throttle_pedal_actual:.2f}%, "
            f"Fault1: {throttle_flt1_type}, "
            f"Fault2: {throttle_flt2_type}, "
            f"En State: {throttle_en_state}"
        )
        self.vehicle_status.throttle = round(throttle_pedal_actual, 2)
        self.vehicle_status.timestamp = time.time()

    def on_brake_report_501(self, data):
        """
        处理 Brake Report (0x501) 消息。
        """

        brake_pedal_actual = (data[3] << 8 | data[4]) * 0.1
        brake_flt2_type = data[2]
        brake_flt1_type = data[1]
        # 0: 'MANUAL', 1: 'AUTO', 2: 'TAKEOVER', 3: 'STANDBY'
        brake_en_state = data[0] & 0x03

        self.log.debug(
            f"Brake Report (0x501): "
            f"Pedal: {brake_pedal_actual:.2f}%, "
            f"Fault1: {brake_flt1_type}, "
            f"Fault2: {brake_flt2_type}, "
            f"En_State: {brake_en_state}"
        )
        self.vehicle_status.brake = round(brake_pedal_actual, 2)
        self.vehicle_status.timestamp = time.time()

    def on_steering_report_502(self, data):
        """
        处理 Steering Report (0x502) 消息。
        """
        steer_angle_spd_actual = data[7]
        steer_flt2_type = data[2]
        steer_flt1_type = data[1]
        # 0: 'MANUAL', 1: 'AUTO', 2: 'TAKEOVER', 3: 'STANDBY'
        steer_en_state = data[0] & 0x03
        steer_angle_actual =(data[3] << 8 | data[4]) - 500.0

        self.log.debug(
            f"Steering Report (0x502): "
            f"Steering Angle: {steer_angle_actual}, "
            f"Steering_Speed: {steer_angle_spd_actual}, "
            f"Fault1: {steer_flt1_type}, "
            f"Fault2: {steer_flt2_type}, "
            f"En State: {steer_en_state}"
        )

        steering_in_perc = round(steer_angle_actual / 5.0, 2)
        steering_in_perc = max(-100, min(steering_in_perc, 100))
        self.vehicle_status.steering = steering_in_perc
        self.vehicle_status.timestamp = time.time()

    def on_gear_report_503(self, data):
        """
        处理 Gear Report (0x503) 消息。
        """
        gear_flt_type = data[1]
        gear_actual = data[0] & 0x07

        self.log.debug(
            f"Gear Report (0x503): " f"Gear: {gear_actual}, " f"Fault: {gear_flt_type}"
        )
        self.vehicle_status.gear = self.get_gear_state(gear_actual)
        self.vehicle_status.timestamp = time.time()

    def on_park_report_504(self, data):
        """
        处理 Park Report (0x504) 消息。
        """
        parking_actual = data[0] & 0x01
        parking_flt = data[1]

        self.log.debug(
            f"Parking Report (0x504): "
            f"parking: {parking_actual}, "
            f"Fault: {parking_flt}"
        )
        self.vehicle_status.parking_brake = 1 if parking_actual else 0
        self.vehicle_status.timestamp = time.time()

    def on_vcu_report_505(self, data):
        """
        处理 VCU Report (0x505) 消息。
        """
        brake_light_actual = (data[1] >> 3) & 0x01
        turn_light_actual = data[7] & 0x03
        chassis_errcode = data[5]
        drive_mode_sts = (data[4] >> 5) & 0x07
        steer_mode_sts = data[1] & 0x07
        # 0: 'MANUAL_REMOTE', 1: 'AUTO', 2: 'EMERGENCY', 3: 'STANDBY'
        vehicle_mode_state = (data[4] >> 3) & 0x03
        frontcrash_state = (data[4] >> 1) & 0x01
        backcrash_state = (data[4] >> 2) & 0x01
        aeb_state = data[4] & 0x01

        # 拼接成一个12位的整数
        x = (data[0] << 4) | (data[1] >> 4) & 0x0F
        # 将12位整数转换为有符号整数
        if x & 0x800:  # 检查符号位
            x -= 0x1000  # 转换为负数
        # 转换为浮点数
        vehicle_acc = x * 0.010000
        vehicle_acc = max(-10, min(vehicle_acc, 10))  # 限制 vehicle_acc 在 -10 到 10 的范围内

        # 拼接成一个16位的整数
        x = (data[2] << 8 | data[3])
        # 将16位整数转换为有符号整数
        # 16位有符号整数的最大值为 32767 (0x7FFF)，最小值为 -32768 (0x8000)
        if x & 0x8000:  # 检查符号位
            x -= 0x10000  # 转换为负数  
        vehicle_speed = x * 0.001
        vehicle_speed = max(-32.768, min(vehicle_speed, 32.767))  # 限制 vehicle_speed 在 -32.768 到 32.767 的范围内

        self.log.debug(
            f"VCU Report (0x505): "
            f"Brake Light: {brake_light_actual}, "
            f"Turn Light: {turn_light_actual}, "
            f"Chassis Error: {chassis_errcode}, "
            f"Drive Mode: {drive_mode_sts}, "
            f"Steer Mode: {steer_mode_sts}, "
            f"Vehicle Mode: {vehicle_mode_state}, "
            f"Front Crash: {frontcrash_state}, "
            f"Back Crash: {backcrash_state}, "
            f"AEB State: {aeb_state}, "
            f"Acceleration: {vehicle_acc:.2f} m/s^2, "
            f"Speed: {vehicle_speed:.2f} m/s"
        )

        self.vehicle_status.speed = round(vehicle_speed * 3.6, 2)   # km/h
        self.vehicle_status.turn_light = self.get_turn_light_state(turn_light_actual)
        self.vehicle_status.driving_mode = self.get_driving_mode(vehicle_mode_state)
        self.vehicle_status.timestamp = time.time()

    def on_wheel_speed_report_506(self, data):
        """
        处理 Wheel Speed Report (0x506) 消息。
        """
        # 解析数据
        front_left_wheel_speed = (data[0] << 8 | data[1]) * 0.001
        front_right_wheel_speed = (data[2] << 8 | data[3]) * 0.001
        rear_left_wheel_speed = (data[4] << 8 | data[5]) * 0.001
        rear_right_wheel_speed = (data[6] << 8 | data[7]) * 0.001

        # 打印或发布消息
        self.log.debug(
            f"Wheel Speed Report (0x506): "
            f"Front Left: {front_left_wheel_speed:.3f} m/s, "
            f"Front Right: {front_right_wheel_speed:.3f} m/s, "
            f"Rear Left: {rear_left_wheel_speed:.3f} m/s, "
            f"Rear Right: {rear_right_wheel_speed:.3f} m/s"
        )

    def on_ultr_sensor_507(self, data):
        """
        处理 Ultrasonic Sensor Report (0x507) 消息。
        """
        # uiuss9_tof_direct, 0-65535 cm (ultrasonic distance)
        uiuss9_tof_direct = ((data[2] << 8) | data[3]) * 0.017240
        # uiuss8_tof_direct, 0-65535 cm (ultrasonic distance)
        uiuss8_tof_direct = ((data[0] << 8) | data[1]) * 0.017240
        # uiuss11_tof_direct, 0-65535 cm (ultrasonic distance)
        uiuss11_tof_direct = ((data[6] << 8) | data[7]) * 0.017240
        # uiuss10_tof_direct, 0-65535 cm (ultrasonic distance)
        uiuss10_tof_direct = ((data[4] << 8) | data[5]) * 0.017240

        self.log.debug(
            f"Ultrasonic Sensor Report (0x507): "
            f"UIUSS8: {uiuss8_tof_direct:.2f} cm, "
            f"UIUSS9: {uiuss9_tof_direct:.2f} cm, "
            f"UIUSS10: {uiuss10_tof_direct:.2f} cm, "
            f"UIUSS11: {uiuss11_tof_direct:.2f} cm"
        )

    def on_ultr_sensor_508(self, data):
        """
        处理 Ultrasonic Sensor Report (0x508) 消息。
        """
        uiuss9_tof_indirect = ((data[2] << 8) | data[3]) * 0.017240
        uiuss8_tof_indirect = ((data[0] << 8) | data[1]) * 0.017240
        uiuss11_tof_indirect = ((data[6] << 8) | data[7]) * 0.017240
        uiuss10_tof_indirect = ((data[4] << 8) | data[5]) * 0.017240

        self.log.debug(
            f"Ultrasonic Sensor Report (0x508): "
            f"UIUSS8: {uiuss8_tof_indirect:.2f} cm, "
            f"UIUSS9: {uiuss9_tof_indirect:.2f} cm, "
            f"UIUSS10: {uiuss10_tof_indirect:.2f} cm, "
            f"UIUSS11: {uiuss11_tof_indirect:.2f} cm"
        )

    def on_ultr_sensor_509(self, data):
        """
        处理 Ultrasonic Sensor Report (0x509) 消息。
        """
        uiuss5_tof_direct = ((data[6] << 8) | data[7]) * 0.017240
        uiuss4_tof_direct = ((data[4] << 8) | data[5]) * 0.017240
        uiuss3_tof_direct = ((data[2] << 8) | data[3]) * 0.017240
        uiuss2_tof_direct = ((data[0] << 8) | data[1]) * 0.017240

        self.log.debug(
            f"Ultrasonic Sensor Report (0x509): "
            f"UIUSS2: {uiuss2_tof_direct:.2f} cm, "
            f"UIUSS3: {uiuss3_tof_direct:.2f} cm, "
            f"UIUSS4: {uiuss4_tof_direct:.2f} cm, "
            f"UIUSS5: {uiuss5_tof_direct:.2f} cm"
        )

    def on_ultr_sensor_510(self, data):
        """
        处理 Ultrasonic Sensor Report (0x510) 消息。
        """
        uiuss5_tof_indirect = ((data[6] << 8) | data[7]) * 0.017240
        uiuss4_tof_indirect = ((data[4] << 8) | data[5]) * 0.017240
        uiuss3_tof_indirect = ((data[2] << 8) | data[3]) * 0.017240
        uiuss2_tof_indirect = ((data[0] << 8) | data[1]) * 0.017240

        self.log.debug(
            f"Ultrasonic Sensor Report (0x510): "
            f"UIUSS2: {uiuss2_tof_indirect:.2f} cm, "
            f"UIUSS3: {uiuss3_tof_indirect:.2f} cm, "
            f"UIUSS4: {uiuss4_tof_indirect:.2f} cm, "
            f"UIUSS5: {uiuss5_tof_indirect:.2f} cm"
        )

    def on_ultr_sensor_511(self, data):
        """
        处理 Ultrasonic Sensor Report (0x511) 消息。
        """
        uiuss7_tof_direct = ((data[6] << 8) | data[7]) * 0.017240
        uiuss6_tof_direct = ((data[4] << 8) | data[5]) * 0.017240
        uiuss1_tof_direct = ((data[2] << 8) | data[3]) * 0.017240
        uiuss0_tof_direct = ((data[0] << 8) | data[1]) * 0.017240

        self.log.debug(
            f"Ultrasonic Sensor Report (0x511): "
            f"uiuss0_tof_direct: {uiuss0_tof_direct:.2f} cm, "
            f"uiuss1_tof_direct: {uiuss1_tof_direct:.2f} cm, "
            f"uiuss6_tof_direct: {uiuss6_tof_direct:.2f} cm, "
            f"uiuss7_tof_direct: {uiuss7_tof_direct:.2f} cm"
        )

    def on_bms_report_512(self, data):
        """
        处理 BMS Report (0x512) 消息。
        """
        
        # battery_current = struct.unpack_from("<h", data, 2)[0] * 0.1
        
        battery_voltage = ((data[0] << 8) | data[1]) * 0.01
        battery_current = ((data[2] << 8) | data[3]) * 0.1 - 3200.0
        battery_soc = int(min(max(data[4], 0), 100))

        self.log.debug(
            f"BMS Report (0x512): "
            f"Voltage: {battery_voltage:.2f} V, "
            f"Current: {battery_current:.2f} A, "
            f"SOC: {battery_soc}%"
        )
        self.vehicle_status.battery = round(battery_soc, 2)

    def on_vin_report_514(self, data):
        """
        处理 VIN Report (0x514) 消息。
        """
        vin07 = data[7]
        vin06 = data[6]
        vin05 = data[5]
        vin04 = data[4]
        vin03 = data[3]
        vin02 = data[2]
        vin01 = data[1]
        vin00 = data[0]

        self.log.debug(f"VIN Report (0x514): VIN07-VIN00: {vin07},{vin06},{vin05},{vin04},{vin03},{vin02},{vin01},{vin00}")

    def on_vin_report_515(self, data):
        """
        处理 VIN Report (0x515) 消息。
        """
        vin15 = data[7]
        vin14 = data[6]
        vin13 = data[5]
        vin12 = data[4]
        vin11 = data[3]
        vin10 = data[2]
        vin09 = data[1]
        vin08 = data[0]

        vin_report515 = {
            "vin15": vin15,
            "vin14": vin14,
            "vin13": vin13,
            "vin12": vin12,
            "vin11": vin11,
            "vin10": vin10,
            "vin09": vin09,
            "vin08": vin08
        }

        self.log.debug(f"VIN Report (0x515): VIN15-VIN08: {vin15},{vin14},{vin13},{vin12},{vin11},{vin10},{vin09},{vin08}")

    def on_vin_report_516(self, data):
        """
        处理 VIN Report (0x516) 消息。
        """
        vin16 = data[0]
        self.log.debug(f"VIN Report (0x516): VIN16: {vin16}")

    @staticmethod
    def get_turn_light_state(state_code):
        states = {0: "None", 1: "Left", 2: "Right", 3: "Hazard"}
        return states.get(state_code, "Unknown")

    @staticmethod
    def get_driving_mode(mode_code):
        modes = {0: "MANUAL", 1: "AUTO", 2: "EMERGENCY", 3: "STANDBY"}
        return modes.get(mode_code)

    @staticmethod
    def get_gear_state(gear_code):
        gears = {1: "P", 2: "R", 3: "N", 4: "D"}
        return gears.get(gear_code, "Unknown")

    def get_vehicle_status(self):
        self.log.debug(f"update vehicle status, time:{self.vehicle_status.timestamp}, speed:{self.vehicle_status.speed}, throttle:{self.vehicle_status.throttle}, brake:{self.vehicle_status.brake}, steering:{self.vehicle_status.steering}, gear:{self.vehicle_status.gear}, park_braking:{self.vehicle_status.parking_brake}, driving_mode: {self.vehicle_status.driving_mode}")
        return self.vehicle_status


class HOOKE2CanCommandHandler:
    def __init__(self, log):
        # 默认命令值
        # 0x100
        self.throttle_pedal_target = 0
        # 0x101
        self.brake_pedal_target = 0
        self.brake_en_ctrl = int(0)
        # 0x102
        self.steer_en_ctrl = int(0)
        self.steer_angle_target = 0
        # 0x103
        self.gear_target = int(3)
        self.gear_en_ctrl = int(0)
        # 0x104
        self.park_target = int(0)
        self.park_en_ctrl = int(0)
        # 0x105
        self.turn_light_ctrl = int(0)
        self.vin_req = int(0)
        self.drive_mode_ctrl = int(0)
        self.steer_mode_ctrl = int(0)

        self.reset_can_msg = False      # 是否重置 CAN message
        self.log = log

    def set_auto_drive(self, auto_drive = None):
        """
        强制重置所有 CAN 指令为默认值
        """
        if auto_drive == 1:
            # enable AUTO drive
            self.reset_can_msg = False
            # self.log.debug("Set AUTO drive.")
        elif auto_drive == 0 or None: 
            # disable AUTO drive / MANUAL or other
            self.reset_can_msg = True
            self.log.debug("Reset all CAN commands to default value.")

    def send_throttle_command(self, enable=0, throttle_cmd=0):
        """
        油门can message (0x100)
        """
        if self.reset_can_msg is True:
            # reset can message
            enable=0
            throttle_cmd=0

        msg = can.Message(arbitration_id=0x100, dlc=8, is_extended_id=False)
        msg.timestamp = time.time()  
        msg.data = self._pack_throttle_data(self, enable, throttle_cmd)
        return msg

    def send_brake_command(self, enable=0, brake_cmd=0):
        """
        制动can message (0x101)
        """
        if self.reset_can_msg is True:
            # reset can message
            enable=0
            brake_cmd=0

        msg = can.Message(arbitration_id=0x101, dlc=8, is_extended_id=False)
        msg.timestamp = time.time()  
        msg.data = self._pack_brake_data(self, enable, brake_cmd)
        return msg

    def send_steering_command(self, enable=0, steering_cmd=0):
        """
        转向can message (0x102)
        """
        if self.reset_can_msg is True:
            # reset can message
            enable=0
            steering_cmd=0
            
        msg = can.Message(arbitration_id=0x102, dlc=8, is_extended_id=False)
        msg.timestamp = time.time()  
        msg.data = self._pack_steering_data(self, enable, steering_cmd)
        return msg

    def send_gear_command(self, enable=0, gear_cmd=4):
        """
        档位can message (0x103)
        """
        if self.reset_can_msg is True:
            # reset can message
            enable=0
            gear_cmd=0

        msg = can.Message(arbitration_id=0x103, dlc=8, is_extended_id=False)
        msg.timestamp = time.time()  
        msg.data = self._pack_gear_data(self, enable, gear_cmd)
        return msg

    def send_park_command(self, enable=0, park_cmd=0):
        """
        驻车制动can message (0x104)
        """
        if self.reset_can_msg is True:
            # reset can message
            enable=0
            park_cmd=0

        msg = can.Message(arbitration_id=0x104, dlc=8, is_extended_id=False)
        msg.timestamp = time.time()  
        msg.data = self._pack_park_data(self, enable, park_cmd)
        return msg

    def send_vehicle_mode_command(self, light_cmd=0, vin_req=0, drive_mode=0, steer_mode=0):
        """
        车辆模式can message (0x105)
        """
        if self.reset_can_msg is True:
            # reset can message
            light_cmd=0
            vin_req=0
            drive_mode=0
            steer_mode=0

        msg = can.Message(arbitration_id=0x105, dlc=8, is_extended_id=False)
        msg.timestamp = time.time()     
        msg.data = self._pack_vehicle_mode_data(self, light_cmd, vin_req, drive_mode, steer_mode)
        return msg

    @staticmethod
    def _pack_throttle_data(self, enable, cmd):
        """
        油门can message数据
        """
        throttle_en_ctrl = enable           # 默认值：禁用
        vel_target = 0.0                    # 目标速度
        throttle_acc = 0.0                  # 加速度
        self.throttle_pedal_target = cmd    # 踏板目标值
        checksum_100 = 0                    # 校验和

        data = bytearray(8)  # 创建一个 8 字节的字节数组

        # 设置使能
        throttle_en_ctrl = throttle_en_ctrl & 0x01  # 限制为 0 或 1
        data[0] = throttle_en_ctrl & 0x01

        # 设置目标速度
        vel_target = min(max(0.0, vel_target / 4.0), 10.23) # 限制范围 [0, 10.23]
        vel_target_int = int(vel_target / 0.01)  # 转换为整数
        data[5] = vel_target_int & 0xFF
        data[6] = (vel_target_int >> 8) & 0x03

        # 设置加速度
        throttle_acc = min(max(0.0, throttle_acc), 10.0)  # 限制范围 [0, 10.0]
        throttle_acc_int = int(throttle_acc / 0.01)  # 转换为整数
        data[1] = throttle_acc_int & 0xFF
        data[2] = (throttle_acc_int >> 8) & 0x03

        # 设置踏板目标值
        throttle_pedal_target = min(max(0.0, self.throttle_pedal_target), 100.0)  # 限制范围 [0, 100.0]
        throttle_pedal_int = int(throttle_pedal_target / 0.1)
        data[3] = (throttle_pedal_int >> 8) & 0xFF
        data[4] = throttle_pedal_int & 0xFF

        # 设置校验和
        # checksum_100 = sum(data[:7]) & 0xFF  # 计算校验和
        data[7] = checksum_100
        self.log.debug(f'throttle en:{data[0]}, brake_pedal:{throttle_pedal_int}, {data[3],data[4]}')
        return data

    @staticmethod
    def _pack_brake_data(self, enable, cmd):
        """
        制动can message数据
        """
        aeb_en_ctrl = 0                 # 默认值：AEB 禁用
        brake_dec = 0.25                # 制动减速度
        self.brake_pedal_target = cmd   # 制动踏板目标值
        self.brake_en_ctrl = enable     # 制动使能
        checksum_101 = 0                # 校验和

        data = bytearray(8)  # 创建一个 8 字节的字节数组

        # 设置 AEB 使能
        data[0] = (aeb_en_ctrl << 1) | (self.brake_en_ctrl & 0x01)

        # 设置制动减速度
        brake_dec_int = int(brake_dec / 0.01)  # 转换为整数
        data[1] = brake_dec_int & 0xFF
        data[2] = (brake_dec_int >> 8) & 0x03

        # 设置制动踏板目标值
        brake_pedal_int = int(self.brake_pedal_target / 0.1)
        data[3] = (brake_pedal_int >> 8) & 0xFF
        data[4] = brake_pedal_int & 0xFF

        # 设置校验和
        data[7] = checksum_101
        self.log.debug(f'brake en:{data[0]}, brake_pedal:{brake_pedal_int}, {data[3],data[4]}')
        return data

    @staticmethod
    def _pack_steering_data(self, enable, cmd):
        """
        转向can message数据
        """
        self.steer_en_ctrl = enable         # 默认值：转向使能禁用
        self.steer_angle_target = cmd/100.0 * 500  # 目标转向角度, degree
        steer_angle_spd = 250               # 转向速度, degree/s
        checksum_102 = 0                    # 校验和

        data = bytearray(8)  # 创建一个 8 字节的字节数组

        # 设置转向使能
        steer_en_ctrl = self.steer_en_ctrl & 0x01  # 限制为 0 或 1
        data[0] = steer_en_ctrl & 0x01

        # 设置目标转向角度
        # 偏移量 -500
        steer_angle_target = max(-500, min(500, self.steer_angle_target))
        angle_int = int(steer_angle_target + 500)
        data[3] = (angle_int >> 8) & 0xFF
        data[4] = angle_int & 0xFF

        # 设置转向速度
        # 范围 [0, 250]
        steer_angle_spd = max(0, min(250, steer_angle_spd))
        data[1] = steer_angle_spd & 0xFF

        # 设置校验和
        # checksum_102 = sum(data[:7]) & 0xFF  # 计算校验和
        data[7] = checksum_102

        self.log.debug(f'steering en:{data[0]}, steer angle target:{steer_angle_target}, {data[3],data[4]}')
        return data

    @staticmethod
    def _pack_gear_data(self, enable, cmd):
        """
        档位can message数据
        """
        self.gear_target = cmd      # 默认值：D
        self.gear_en_ctrl = enable  # 默认值：挡位使能/禁用
        checksum_103 = 0            # 校验和

        data = bytearray(8)  # 创建一个 8 字节的字节数组

        # 设置挡位使能
        gear_en_ctrl = self.gear_en_ctrl & 0x01  # 限制为 0 或 1
        data[0] = gear_en_ctrl & 0x01

        # 设置挡位目标
        gear_target = self.gear_target & 0x07  # 限制为 0-7
        data[1] = gear_target & 0x0F  # 档位目标占低 4 位

        # 设置校验和
        data[7] = checksum_103
        self.log.debug(f'gear en:{data[0]}, gear target:{gear_target}, {data[1]}')
        return data

    @staticmethod
    def _pack_park_data(self, enable, cmd):
        """
        驻车制动can message数据
        """
        checksum_104 = 0
        self.park_target = cmd       # 默认值：释放驻车
        self.park_en_ctrl = enable   # 默认值：驻车使能/禁用

        data = bytearray(8)  # 创建一个 8 字节的字节数组

        # 设置驻车使能
        park_en_ctrl = self.park_en_ctrl & 0x01  # 限制为 0 或 1
        data[0] = park_en_ctrl & 0x01

        # 设置驻车目标
        park_target = self.park_target & 0x01  # 限制为 0 或 1, 0: 'PARK_TARGET_RELEASE', 1: 'PARK_TARGET_PARKING_TRIGGER
        data[1] = park_target

        # 设置校验和
        # checksum_104 = sum(data[:7]) & 0xFF  # 计算校验和
        data[7] = checksum_104
        self.log.debug(f'gear en:{data[0]}, park target:{park_target}, {data[1]}')
        return data

    @staticmethod
    def _pack_vehicle_mode_data(self, light_cmd, vin_req, drive_mode, steer_mode):
        """
        车辆模式can message数据
        """
        checksum_105 = 0
        self.turn_light_ctrl = 0     # 默认值：转向灯关闭, off-0, left-1, right-2, harzad-3
        self.vin_req = 0             # 默认值：VIN 请求, disable-0, enable-1
        self.drive_mode_ctrl = 0     # 默认值：驾驶模式, throttle pedal-0, speed-1
        self.steer_mode_ctrl = 0     # 默认值：标准转向模式, front wheel-0, four-wheel-non-direction-1, four-wheel-sync-direction-2

        data = bytearray(8)  # 创建一个 8 字节的字节数组

        # 设置转向模式控制
        steer_mode_ctrl = self.steer_mode_ctrl & 0x07  # 限制为 0-7
        data[0] = (steer_mode_ctrl & 0x07)

        # 设置驾驶模式控制
        drive_mode_ctrl = self.drive_mode_ctrl & 0x07  # 限制为 0-7
        data[1] = (drive_mode_ctrl & 0x07) << 5

        # 设置转向灯控制
        turn_light_ctrl = self.turn_light_ctrl & 0x07  # 限制为 0-7
        data[2] = (turn_light_ctrl & 0x03) << 6

        # 设置 VIN 请求
        vin_req = vin_req & 0x01  # 限制为 0 或 1
        data[3] = (vin_req & 0x01) << 7

        # 设置校验和
        # checksum_105 = sum(data[:7]) & 0xFF  # 计算校验和
        data[7] = checksum_105
        self.log.debug(f'steer_mode_ctrl:{data[0]}, drive_mode_ctrl:{data[1]}, turn_light_ctrl:{data[2]}, vin_req:{data[3]}')
        return data
    
