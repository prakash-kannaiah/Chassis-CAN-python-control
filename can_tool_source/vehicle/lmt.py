#!/usr/bin/env python
# -*- coding: utf-8 -*-

import struct
import can
import time
import math

from vehicle.vehicle_status import VehicleStatus

LMT_SEND_PERIOD = 0.01  # 发送周期（单位：秒）

# 创建CAN解析类
class LMTCanReportHandler:
    def __init__(self, log):
        self.log = log
        self.vehicle_status = VehicleStatus()
        # 定义目标 ID 和对应的处理函数
        self.lmt_can_report_ids = {
            0x620: self.on_motor_fb1_620,
            0x621: self.on_motor_fb1_621,
            0x622: self.on_motor_fb2_622,
            0x623: self.on_motor_fb2_623,
        }

    def handle_message(self, msg: can.Message):
        """
        处理 CAN 消息。
        """
        if msg is not None:
            if msg.arbitration_id in self.lmt_can_report_ids:
                self.lmt_can_report_ids[msg.arbitration_id](msg.data)
                
    def on_motor_fb1_620(self, data):
        """
        处理 Motor Feedback 1 (0x620) 消息。
        """
        cur_fb = (data[0] << 8 | data[1]) * 0.0078125
        spd_fb = struct.unpack('>h', data[2:4])[0] * 0.25
        workmod = data[4] & 0x0F
        leg_sta = data[5] & 0x0F
        temp = data[6] * 1.0
        rolling = data[7] 

        self.log.debug(
            f"Motor Feedback 1 (0x620): "
            f"CurFb: {cur_fb:.2f} A, "
            f"SpdFb: {spd_fb:.2f} rpm, "
            f"WorkMod: {workmod}, "
            f"LegSta: {leg_sta}, "
            f"Temp: {temp:.2f} °C, "
            f"Rolling: {rolling}"
        )

        self.vehicle_status.motor1_current = cur_fb
        self.vehicle_status.motor1_speed = spd_fb
        self.motor1_work_mode = workmod
        self.motor1_remote_status = leg_sta
        self.motor1_temperature = temp
        self.vehicle_status.timestamp = time.time()

    def on_motor_fb1_621(self, data):
        """
        处理 Motor Feedback 1 (0x621) 消息。
        """
        cur_fb = (data[0] << 8 | data[1]) * 0.0078125
        spd_fb = struct.unpack('>h', data[2:4])[0] * 0.25
        workmod = data[4] & 0x0F
        leg_sta = data[5] & 0x0F
        temp = data[6] * 1.0
        rolling = data[7] 

        self.log.debug(
            f"Motor Feedback 1 (0x621): "
            f"CurFb: {cur_fb:.2f} A, "
            f"SpdFb: {spd_fb:.2f} rpm, "
            f"WorkMod: {workmod}, "
            f"LegSta: {leg_sta}, "
            f"Temp: {temp:.2f} °C, "
            f"Rolling: {rolling}"
        )
        self.vehicle_status.motor2_current = cur_fb
        self.vehicle_status.motor2_speed = spd_fb
        self.motor2_work_mode = workmod
        self.motor2_remote_status = leg_sta
        self.motor2_temperature = temp
        self.vehicle_status.timestamp = time.time()

    def on_motor_fb2_622(self, data):
        """
        处理 Motor Feedback 2 (0x622) 消息。
        """
        circles = (data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]) * 1.0
        rolling = data[4] & 0x0F

        self.log.debug(
            f"Motor Feedback 2 (0x622): "
            f"Circles: {circles:.6f} revolutions, "
            f"Rolling: {rolling}"
        )
        self.motor1_pulse_count = circles
        self.vehicle_status.timestamp = time.time()

    def on_motor_fb2_623(self, data):
        """
        处理 Motor Feedback 2 (0x623) 消息。
        """
        circles = (data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]) * 1.0
        rolling = data[4] & 0x0F

        self.log.debug(
            f"Motor Feedback 2 (0x623): "
            f"Circles: {circles:.6f} revolutions, "
            f"Rolling: {rolling}"
        )
        self.motor2_pulse_count = circles
        self.vehicle_status.timestamp = time.time()
    
    def get_vehicle_status(self):
        self.log.debug(f"update vehicle status, time:{self.vehicle_status.timestamp}")
        return self.vehicle_status

class LMTCanCommandHandler:
    def __init__(self, log):
        # 默认命令值
        self.wheelbase = 1.0
        # 0x520 
        self.motor1_workmod_req = 0
        self.motor1_target_spd = 0.0
        self.motor1_target_cur = 0.0
        self.motor1_rolling = 0

        # 0x521
        self.motor2_workmod_req = 0.0
        self.motor2_target_spd = 0.0
        self.motor2_target_cur = 0.0
        self.motor2_rolling = 0

        self.reset_can_msg = False      # 是否重置 CAN message
        self.log = log

    def send_drive_command(self, speed, steering_angle, rolling):
        """
        发送直行+转向命令。
        :param speed: 基准速度（单位：rpm）
        :param steering_angle: 转向角度（单位：度）
        """
        # 将转向角度转换为弧度
        steering_angle_rad = math.radians(steering_angle)

        # 计算外侧轮和内侧轮的速度
        v_outer = speed + (speed * math.tan(steering_angle_rad) * self.wheelbase / 2)
        v_inner = speed - (speed * math.tan(steering_angle_rad) * self.wheelbase / 2)
        # print(f'v_out: {v_outer}, v_inner: {v_inner}')

        # 根据转向方向分配速度
        if steering_angle > 0:  # turn right
            motor1_speed = v_outer  # 电机1
            motor2_speed = v_inner  # 电机2
        elif steering_angle < 0:  # turn left
            motor1_speed = v_outer  # 电机1
            motor2_speed = v_inner  # 电机2
        else:  # 直行
            motor1_speed = speed
            motor2_speed = speed

        # 发送电机控制指令
        msg1 = self.send_motor_ctrlcmd_520(2, motor1_speed, 0.0, rolling)
        msg2 = self.send_motor_ctrlcmd_521(2, motor2_speed, 0.0, rolling)

        return msg1, msg2
        
    def send_motor_ctrlcmd_520(self, workmod_req=0, target_spd=0.0, target_cur=0.0, rolling=0):
        """
        发送电机控制指令 (0x520)。
        """
        if self.reset_can_msg is True:
            # reset can message
            workmod_req = 0
            target_spd = 0.0
            target_cur = 0.0
            rolling = 0

        msg = can.Message(arbitration_id=0x520, dlc=8, is_extended_id=False)
        msg.timestamp = time.time()  
        msg.data = self._pack_motor_ctrlcmd_data(self, workmod_req=workmod_req, target_spd=-target_spd, target_cur=-target_cur, rolling=rolling, motor_num=1)
        return msg

    def send_motor_ctrlcmd_521(self, workmod_req=0, target_spd=0, target_cur=0, rolling=0):
        """
        发送电机控制指令 (0x521)。
        """
        if self.reset_can_msg is True:
            # reset can message
            workmod_req = 0
            target_spd = 0.0
            target_cur = 0.0
            rolling = 0

        msg = can.Message(arbitration_id=0x521, dlc=8, is_extended_id=False)
        msg.timestamp = time.time()  
        msg.data = self._pack_motor_ctrlcmd_data(self, workmod_req=workmod_req, target_spd=target_spd, target_cur=target_cur, rolling=rolling, motor_num=2)
        return msg

    @staticmethod
    def _pack_motor_ctrlcmd_data(self, workmod_req, target_spd, target_cur, rolling, motor_num):
        """
        打包电机控制指令数据。
        """
         # 限制 target_spd 在 -3000 到 3000 之间
        target_spd = max(-3000, min(target_spd, 3000))
        # 限制 target_cur 在 -80 到 80 之间
        target_cur = max(-80, min(target_cur, 80))

        if motor_num == 1:
            self.motor1_workmod_req = workmod_req
            self.motor1_target_spd = target_spd
            self.motor1_target_cur = target_cur
            self.motor1_rolling = rolling
        elif motor_num == 2:
            self.motor2_workmod_req = workmod_req
            self.motor2_target_spd = target_spd
            self.motor2_target_cur = target_cur
            self.motor2_rolling = rolling

        data = bytearray(8)  # 创建一个 8 字节的字节数组

        data[0] = workmod_req & 0x0F

        target_spd_int = int(target_spd / 0.25)
        data[4] = target_spd_int & 0xFF
        data[3] = (target_spd_int >> 8) & 0xFF
        
        target_cur_int = int(target_cur / 0.0078125)
        data[6] = target_cur_int & 0xFF
        data[5] = (target_cur_int >> 8) & 0xFF

        data[7] = rolling 
        self.log.debug(f'workmod: {workmod_req}, target_spd:{target_spd_int}, target_cur:{target_cur_int}, rolling: {rolling}')
        return data
    
