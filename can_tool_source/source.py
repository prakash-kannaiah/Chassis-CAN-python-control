#!/usr/bin/env python
# -*- coding: utf-8 -*-

from vehicle.hooke2 import HOOKE2CanReportHandler
from vehicle.hooke2 import HOOKE2CanCommandHandler
from vehicle.hooke2 import HOOKE2_SEND_PERIOD

from vehicle.lmt import LMTCanReportHandler
from vehicle.lmt import LMTCanCommandHandler
from vehicle.lmt import LMT_SEND_PERIOD

from vehicle.vehicle_status import VehicleStatus

import tkinter as tk
from tkinter import ttk
from datetime import datetime
import time
import threading
import logging
import can
from collections import deque
import sys
import signal

TITLE = "Edutech CAN Tool"

PADX = 5
PADY = 5

def detect_pcan_channels(max_channels=4):
    found = []
    for i in range(1, max_channels + 1):
        channel = f'PCAN_USBBUS{i}'  # Dynamically test each channel
        try:
            bus = can.Bus(interface='pcan', channel=channel, bitrate=500000)
            found.append(channel)
            bus.shutdown()
        except can.CanError:
            continue
    return found

class App(object):
    def __init__(self, root, log_level=logging.ERROR):
        logging.basicConfig(level=log_level)
        self.logger = logging.getLogger(__name__)
        self.lock = threading.Lock()

        self.root = root
        self.root.title(TITLE)
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        self.canbus = None
        self.can_connect_status = False
        self.can_recv_status = False
        self.can_send_status = False
        self.can_send_messages = []

        # self.can_report_handler = CanReportHandler(self.logger)
        # self.can_command_handler = CanCommandHandler(self.logger)
        self.can_report_handler = LMTCanReportHandler(self.logger)
        self.can_command_handler = LMTCanCommandHandler(self.logger)

        self.vehicle_info_canvas_initialized = False  # 引入标志变量

        self.setup()

        self.thread_event = threading.Event()
        self.thread_event.set()

        self.send_period = 0.02
        self.recv_thread = threading.Thread(target=self.recv_threading_handler)
        self.send_thread = threading.Thread(target=self.send_threading_handler)
        self.update_vehicle_data_thread = threading.Thread(target=self.update_vehicle_data_handler)
        self.recv_thread.start()
        self.send_thread.start()
        self.update_vehicle_data_thread.start()

    def update_vehicle_info_handler(self):
        if not self.vehicle_info_canvas_initialized:
            self.logger.warning("vehicle_info_canvas is not initialized or has been destroyed. Skipping update.")
            return

        # print(f'update_vehicle_info_handler status: {self.can_report_handler.get_vehicle_status()}')
        if self.can_recv_status:
            vehicle_status = self.can_report_handler.get_vehicle_status()

            self.vehicle_current_driving_mode_info.delete("1.0", "end")
            self.vehicle_current_driving_mode_info.insert("end", f"{vehicle_status.driving_mode}\n")
            
            self.vehicle_current_gear_info.delete("1.0", "end")
            self.vehicle_current_gear_info.insert("end", f"{vehicle_status.gear}\n")
            
            self.vehicle_current_throttle_info.delete("1.0", "end")
            self.vehicle_current_throttle_info.insert("end", f"{vehicle_status.throttle}\n")
            
            self.vehicle_current_brake_info.delete("1.0", "end")
            self.vehicle_current_brake_info.insert("end", f"{vehicle_status.brake}\n")
            
            self.vehicle_current_steering_info.delete("1.0", "end")
            self.vehicle_current_steering_info.insert("end", f"{vehicle_status.steering}\n")
            
            self.vehicle_current_speed_info.delete("1.0", "end")
            self.vehicle_current_speed_info.insert("end", f"{vehicle_status.speed}\n")

            self.vehicle_current_battery_info.delete("1.0", "end")
            self.vehicle_current_battery_info.insert("end", f"{vehicle_status.battery}\n")

            self.vehicle_current_parking_info.delete("1.0", "end")
            self.vehicle_current_parking_info.insert("end", f"{vehicle_status.parking_brake}\n")

        else:
            logging.warning(f"can receive no data.")

        self.root.after(100, self.update_vehicle_info_handler)
    
    def update_vehicle_info_handler_LMT(self):
        if self.can_recv_status:
            vehicle_status = self.can_report_handler.get_vehicle_status()

            # 更新电机 1 的状态信息
            self.vehicle_current_current_info_1.delete("1.0", "end")
            self.vehicle_current_current_info_1.insert("end", f"{vehicle_status.motor1_current:.2f} A\n")

            self.vehicle_current_speed_info_1.delete("1.0", "end")
            self.vehicle_current_speed_info_1.insert("end", f"{vehicle_status.motor1_speed:.2f} rpm\n")

            self.vehicle_current_mode_info_1.delete("1.0", "end")
            self.vehicle_current_mode_info_1.insert("end", f"{vehicle_status.motor1_work_mode}\n")

            self.vehicle_current_remote_status_info_1.delete("1.0", "end")
            self.vehicle_current_remote_status_info_1.insert("end", f"{vehicle_status.motor1_remote_status}\n")

            self.vehicle_current_temperature_info_1.delete("1.0", "end")
            self.vehicle_current_temperature_info_1.insert("end", f"{vehicle_status.motor1_temperature:.2f} °C\n")

            self.vehicle_current_pulse_count_info_1.delete("1.0", "end")
            self.vehicle_current_pulse_count_info_1.insert("end", f"{vehicle_status.motor1_pulse_count:.6f}\n")

            # 更新电机 2 的状态信息
            self.vehicle_current_current_info_2.delete("1.0", "end")
            self.vehicle_current_current_info_2.insert("end", f"{vehicle_status.motor2_current:.2f} A\n")

            self.vehicle_current_speed_info_2.delete("1.0", "end")
            self.vehicle_current_speed_info_2.insert("end", f"{vehicle_status.motor2_speed:.2f} rpm\n")

            self.vehicle_current_mode_info_2.delete("1.0", "end")
            self.vehicle_current_mode_info_2.insert("end", f"{vehicle_status.motor2_work_mode}\n")

            self.vehicle_current_remote_status_info_2.delete("1.0", "end")
            self.vehicle_current_remote_status_info_2.insert("end", f"{vehicle_status.motor2_remote_status}\n")

            self.vehicle_current_temperature_info_2.delete("1.0", "end")
            self.vehicle_current_temperature_info_2.insert("end", f"{vehicle_status.motor2_temperature:.2f} °C\n")

            self.vehicle_current_pulse_count_info_2.delete("1.0", "end")
            self.vehicle_current_pulse_count_info_2.insert("end", f"{vehicle_status.motor2_pulse_count:.6f}\n")

        else:
            self.logger.warning(f"CAN receive no data.")
            
        self.root.after(100, self.update_vehicle_info_handler_LMT)

    def recv_threading_handler(self):
        while self.thread_event.is_set():
            while self.thread_event.is_set() and self.can_recv_status:
                msg = self.canbus.recv(timeout=0.1)
                self.can_report_handler.handle_message(msg)
                self.logger.debug(f"recv: {msg}")
                self.can_recv_info.insert(tk.END, f"{msg}\n")
                self.can_recv_info.see(tk.END)
                # time.sleep(0.01)

            time.sleep(0.1)

    def send_threading_handler(self):
        while self.thread_event.is_set():
            while self.thread_event.is_set() and self.can_send_status:
                mode = self.mode.get()
                if mode == "Normal":
                    try:
                        id = int(self.can_data_id.get(), 16)
                        times = int(self.can_times.get())
                        interval = int(self.can_interval.get()) / 1_000.0  # Convert to seconds
                        length = int(self.can_data_length.get())
                        increase = self.can_increase_id_check_button.getvar(
                            self.can_increase_id_check_button["variable"]
                        )
                        extend = self.can_extend_mode_check_button.getvar(
                            self.can_extend_mode_check_button["variable"])
                        if extend == "1":
                            extend = True
                        else:
                            extend = False
                        
                        data = []
                        data.append(int(self.can_send_data1.get(), 16))
                        data.append(int(self.can_send_data2.get(), 16))
                        data.append(int(self.can_send_data3.get(), 16))
                        data.append(int(self.can_send_data4.get(), 16))
                        data.append(int(self.can_send_data5.get(), 16))
                        data.append(int(self.can_send_data6.get(), 16))
                        data.append(int(self.can_send_data7.get(), 16))
                        data.append(int(self.can_send_data8.get(), 16))
                        
                        for i in range(times):
                            if self.thread_event.is_set() and self.can_send_status == False or not self.thread_event.is_set():
                                break
                            if increase != '0':
                                current_id = id + i
                            else:
                                current_id = id
                            msg = can.Message(arbitration_id=current_id, data=data,
                                            is_extended_id=False, dlc=length)
                            self.canbus.send(msg, extend)
                            # many time
                            # self.logger.debug(f"send: {msg}")
                            # self.can_send_info.insert(tk.END, f"{msg}\n")
                            # self.can_send_info.see(tk.END)
                            time.sleep(interval)
                        
                        # clear send status
                        self.can_send_status = False
                        self.root.after(0, lambda: self.can_send_button.config(text="Send", bg="white"))
                        self.root.after(0, lambda: self.can_send_button.update())
                        break
                    except Exception as e:
                        curr_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        self.print_status_log(f"{curr_time} Send: Invalid data", level="error")
                        self.logger.error(f"send: Invalid data {e}")
                        time.sleep(0.1)
                        continue
                else:
                    try:
                        start_time = time.perf_counter()
                        with self.lock:
                            can_msgs = self.can_send_messages
                        for can_msg in can_msgs:
                            self.canbus.send(can_msg, timeout=0.001)
                        
                        end_time = time.perf_counter()
                        time.sleep(self.send_period - (end_time - start_time))
                        
                    except Exception as e:
                        curr_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        self.print_status_log(f"{curr_time} Send: Invalid data", level="error")
                        self.logger.error(f"send: Invalid data {e}")
                        time.sleep(self.send_period)

            time.sleep(0.1)

    def update_vehicle_data_handler(self):
        while self.thread_event.is_set():
            self.lmt_send_counter = 0
            while self.thread_event.is_set() and self.can_send_status:
                start_time = time.perf_counter()

                # 获取当前选择的车型
                vehicle_type = self.vehicle_type.get()
                # 根据车型发送对应的控制指令
                if vehicle_type == "Hooke2":
                    self.send_period = HOOKE2_SEND_PERIOD
                    self.send_hooke2_control_commands()
                elif vehicle_type == "LMT":
                    self.send_period = LMT_SEND_PERIOD
                    self.send_lmt_control_commands()
                else:
                    self.logger.error(f"Please select a vehicle type: {vehicle_type}")

                end_time = time.perf_counter()
                elapse = end_time - start_time
                if self.send_period < elapse:
                    continue
                time.sleep(self.send_period - elapse)
            time.sleep(0.01)

    def send_hooke2_control_commands(self):
        # 获取当前的驾驶模式、油门、刹车、转向等信息
        current_driving_mode = 1 if self.vehicle_driving_mode.get() else 0
        current_throttle = int(self.vehicle_throttle.get())
        current_brake = int(self.vehicle_brake.get())
        current_steering = int(self.vehicle_steering.get())
        current_gear = {
            'P': 1,
            'R': 2,
            'N': 3,
            'D': 4
        }.get(self.vehicle_gear.get(), 3)
        park_release = 0
        if current_gear == 1:
            park_release = 1

        # 设置自动驾驶模式
        self.can_command_handler.set_auto_drive(current_driving_mode)

        # 构建并发送控制指令
        can_send_messages = [
            self.can_command_handler.send_throttle_command(1, current_throttle),
            self.can_command_handler.send_brake_command(1, current_brake),
            self.can_command_handler.send_steering_command(1, current_steering),
            self.can_command_handler.send_gear_command(1, current_gear),
            self.can_command_handler.send_park_command(1, park_release),
            self.can_command_handler.send_vehicle_mode_command(0, 0, 0, 0)
        ]

        with self.lock:
            self.can_send_messages = can_send_messages

        for msg in can_send_messages:
            self.logger.debug(f"send: {msg}")
            self.can_send_info.insert(tk.END, f"{msg}\n")
            self.can_send_info.see(tk.END)
    
    def send_lmt_control_commands(self):
        # 获取当前的电机控制模式、目标速度和目标电流
        motor_mode = self.motor_mode.get()
        target_speed = self.target_speed_scale.get()
        target_steer_angle = self.target_steer_angle_scale.get()
        target_current = self.target_current_scale.get()
        can_send_messages = []
        # 根据电机模式发送对应的控制指令
        if motor_mode == "Speed":
            # 构建并发送控制指令
            msg1, msg2 = self.can_command_handler.send_drive_command(target_speed, target_steer_angle, self.lmt_send_counter)
            can_send_messages = [
                msg1,
                msg2 
            ]
        elif motor_mode == "Current":
            # 构建并发送控制指令
            can_send_messages = [
                self.can_command_handler.send_motor_ctrlcmd_520(1, 0.0, target_current, self.lmt_send_counter),
                self.can_command_handler.send_motor_ctrlcmd_521(1, 0.0, target_current, self.lmt_send_counter)
            ]
        else:
            can_send_messages = [
                self.can_command_handler.send_motor_ctrlcmd_520(0, 0.0, 0.0, 0),
                self.can_command_handler.send_motor_ctrlcmd_521(0, 0.0, 0.0, 0)
            ]

        if self.lmt_send_counter < 16:
            self.lmt_send_counter += 1
        else:
            self.lmt_send_counter = 0
        
        with self.lock:
            self.can_send_messages = can_send_messages

        for msg in can_send_messages:
            self.logger.debug(f"send: {msg}")
            self.can_send_info.insert(tk.END, f"{msg}\n")
            self.can_send_info.see(tk.END)

    def spin(self):
        self.root.mainloop()

    def setup(self):
        self.create_base_layer(self.root, 0, 0)
        self.create_send_setting_layer(self.root, 1, 0)
        self.create_vehicle_control_layer(self.root, 2, 0)
        self.create_send_info_layer(self.root, 3, 0)
        self.create_recv_info_layer(self.root, 4, 0)

    def signal_handler(self, _signum, _frame):
        self.logger.info("Received signal, closing...")
        self.on_closing()

    def on_closing(self):
        self.logger.info("Closing...")
        # self.can_connect_status = False
        # self.can_recv_status = False
        # self.can_send_status = False
        
        # 发送终止信号
        self.thread_event.clear()

        # self.recv_thread.join()
        # self.send_thread.join()
        # self.update_vehicle_data_thread.join()

        # if self.canbus is not None:
        #     self.canbus.shutdown()
        #     self.canbus = None
        if self.canbus is not None:
            try:
                self.canbus.shutdown()  # 安全关闭 canbus
            except Exception as e:
                self.logger.error(f"Error shutting down CAN bus: {e}")
            finally:
                self.canbus = None
        self.root.destroy()
        sys.exit(0)

    def create_base_layer(self, root, row=0, column=0):
        base_frame = tk.Frame(root)
        base_frame.grid(row=row, column=column, padx=PADX, pady=PADY, sticky="nsew")

        self.create_base_config_layer(base_frame, 0, 0)
        self.create_base_log_layer(base_frame, 0, 1)
    
    def create_base_config_layer(self, root, row=0, column=0):
        CONNECT_DEVICE = "Connect Device"
        connect_device_frame = tk.LabelFrame(root, text=CONNECT_DEVICE)
        connect_device_frame.grid(row=row, column=column, padx=PADX, pady=PADY, sticky="nsew")

        connect_device_frame.grid_rowconfigure(0, weight=1)
        connect_device_frame.grid_columnconfigure(0, weight=1)

        self.can_device = tk.StringVar()

        available_channels = detect_pcan_channels()  # <- dynamically detect
        if not available_channels:
            available_channels = ["No PCAN channels found"]

        self.can_device_combobox = ttk.Combobox(
            connect_device_frame,
            values=available_channels,
            width=20,
            textvariable=self.can_device,
            state="readonly"
        )
        self.can_device_combobox.grid(row=0, column=0, padx=PADX, pady=PADY, sticky="ew")

        if available_channels and available_channels[0] != "No PCAN channels found":
            self.can_device_combobox.set(available_channels[0])
        else:
            self.can_device_combobox.set("No PCAN channels found")

        self.can_device_combobox.bind("<<ComboboxSelected>>", self.can_device_combobox_select_handler)

        self.can_connect_button = tk.Button(
            connect_device_frame,
            text="Connect",
            command=self.can_connect_button_handler,
            width=10,
            bg="white"
        )
        self.can_connect_button.grid(row=1, column=0, padx=PADX, pady=PADY, sticky="nsew")

        self.can_start_button = tk.Button(
            connect_device_frame,
            text="Receive",
            command=self.can_start_button_handler,
            width=10,
            bg="white"
        )
        self.can_start_button.grid(row=2, column=0, padx=PADX, pady=PADY, sticky="nsew")

    def create_base_log_layer(self, root, row=0, column=0):
        status_log_frame = tk.LabelFrame(root, text="Log")
        status_log_frame.grid(row=row, column=column, padx=PADX, pady=PADY, sticky="nsew")

        self.status_log_text = tk.Text(status_log_frame, height=8, width=88)
        self.status_log_text.grid(row=0, column=0, padx=PADX, pady=PADY, sticky="nsew")

        scrollbar = tk.Scrollbar(status_log_frame, orient=tk.VERTICAL)
        scrollbar.grid(row=0, column=1, padx=PADX, pady=PADY, sticky="nsew")
        self.status_log_text.config(yscrollcommand=scrollbar.set)

    def create_send_setting_layer(self, root, row=0, column=0):
        send_setting_frame = tk.LabelFrame(root, text="Send Setting")
        send_setting_frame.grid(row=row, column=column, padx=PADX, pady=PADY, sticky="nsew")

        self.create_send_setting_base_layer(send_setting_frame, 0, 0)
        separator_vertical = ttk.Separator(send_setting_frame)
        separator_vertical.grid(row=1, column=0, padx=PADX, pady=PADY, sticky="nsew")
        self.create_send_setting_info_layer(send_setting_frame, 2, 0)

    def create_send_setting_base_layer(self, root, row=0, column=0):
        send_setting_base_frame = tk.Frame(root)
        send_setting_base_frame.grid(row=row, column=column, sticky="nsew")
        self.can_extend_mode_check_button = tk.Checkbutton(
            send_setting_base_frame,
            text="Extended Mode",
            selectcolor="white",
            command=self.can_extend_mode_handler,
        )
        self.can_extend_mode_check_button.grid(
            row=0, column=0, sticky="nsew"
        )

        self.can_increase_id_check_button = tk.Checkbutton(
            send_setting_base_frame,
            text="Increase ID",
            selectcolor="white",
            command=self.can_increase_id_handler
        )
        self.can_increase_id_check_button.grid(row=0, column=1, sticky="nsew")

        separator_vertical1 = ttk.Separator(send_setting_base_frame, orient=tk.VERTICAL)
        separator_vertical1.grid(row=0, column=2, padx=PADX, pady=PADY, sticky="nsew")

        interval_frame = tk.Frame(send_setting_base_frame)
        interval_frame.grid(row=0, column=3, sticky="nsew")
        interval_label= tk.Label(
            interval_frame,
            text="Interval(ms):"
        )
        interval_label.grid(row=0, column=0, sticky="nsew")
        self.can_interval = tk.Entry(interval_frame, width=8)
        self.can_interval.grid(row=0, column=1)
        self.can_interval.insert(0, 1000)
        self.can_interval.bind(
            "<Button-1>", self.can_interval_handler
        )

        times_frame = tk.Frame(send_setting_base_frame)
        times_frame.grid(row=0, column=4, sticky="nsew")
        times_label= tk.Label(
            times_frame,
            text="Times:"
        )
        times_label.grid(row=0, column=0, sticky="nsew")
        self.can_times = tk.Entry(times_frame, width=8)
        self.can_times.grid(row=0, column=1)
        self.can_times.insert(0, 1)
        self.can_times.bind(
            "<Button-1>", self.can_times_handler
        )

        separator_vertical2 = ttk.Separator(send_setting_base_frame, orient=tk.VERTICAL)
        separator_vertical2.grid(row=0, column=5, padx=PADX, pady=PADY, sticky="nsew")

        mode_frame = tk.Frame(send_setting_base_frame)
        mode_frame.grid(row=0, column=6, sticky="nsew")
        mode_label = tk.Label(
            mode_frame,
            text="Mode:")
        mode_label.grid(row=0, column=0, sticky="nsew")
        self.mode = tk.StringVar()
        self.mode_check_button = ttk.Combobox(
            mode_frame,
            values=["Normal", "Vehicle"],
            width=8,
            height=4,
            textvariable=self.mode
        )
        self.mode_check_button.grid(row=0, column=1, sticky="nsew")
        self.mode_check_button.set("Normal")
        self.mode_check_button.bind("<<ComboboxSelected>>", self.mode_select_handler)

        # 添加车型下拉菜单
        vehicle_type_frame = tk.Frame(send_setting_base_frame)
        vehicle_type_frame.grid(row=0, column=7, sticky="nsew")
        vehicle_type_label = tk.Label(vehicle_type_frame, text="Vehicle Type:")
        vehicle_type_label.grid(row=0, column=0, sticky="nsew")
        self.vehicle_type = tk.StringVar()
        self.vehicle_type_combobox = ttk.Combobox(
            vehicle_type_frame,
            values=["Hooke2", "LMT"],  # 可以根据需要添加更多车型
            width=10,
            height=4,
            textvariable=self.vehicle_type
        )
        self.vehicle_type_combobox.grid(row=0, column=1, sticky="nsew")
        self.vehicle_type_combobox.set(" ")  # 默认选择  
        self.vehicle_type_combobox.bind("<<ComboboxSelected>>", self.vehicle_type_select_handler)

    def vehicle_type_select_handler(self, _event):
        selected_vehicle = self.vehicle_type.get()
        self.logger.debug(f"Selected vehicle type: {selected_vehicle}")

        # 取消所有未完成的任务
        if hasattr(self, "update_vehicle_info_canvas_id"):
            self.root.after_cancel(self.update_vehicle_info_canvas_id)

        # 清空当前的 vehicle control 布局
        for widget in self.vehicle_control_frame.winfo_children():
            widget.destroy()

        # 根据车型重新构建布局
        if selected_vehicle == "Hooke2":
            self.create_vehicle_control_layout_Hooke2(self.vehicle_control_frame)
        elif selected_vehicle == "LMT":
            self.create_vehicle_control_layout_LMT(self.vehicle_control_frame)
        else:
            curr_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.print_status_log(f"{curr_time} Please select your vehicle type.", level="error")
    
    def create_vehicle_control_layout_Hooke2(self, vehicle_control_frame):
        self.create_vehicle_control_base_layer(vehicle_control_frame, 0, 0)

        separator_vertical = ttk.Separator(vehicle_control_frame, orient=tk.VERTICAL)
        separator_vertical.grid(row=0, column=1, padx=PADX, pady=PADY, sticky="nsew")
        self.create_vehicle_control_info_layer(vehicle_control_frame, 0, 2)
        
        separator_horizontal = ttk.Separator(vehicle_control_frame, orient=tk.HORIZONTAL)
        separator_horizontal.grid(row=1, column=0, columnspan=3, padx=PADX, pady=PADY, sticky="nsew")
        self.create_vehicle_current_info_layer(vehicle_control_frame, row=2, column=0, columnspan=3)

    def create_vehicle_control_layout_LMT(self, vehicle_control_frame):
        # 清空当前的 vehicle control 布局
        for widget in vehicle_control_frame.winfo_children():
            widget.destroy()
        # 创建电机控制的基本布局
        self.create_vehicle_control_base_layer_LMT(vehicle_control_frame, 0, 0)
        # 创建电机状态信息的布局
        self.create_vehicle_current_info_layer_LMT(vehicle_control_frame, row=2, column=0, columnspan=3)

    def create_send_setting_info_layer(self, root, row, column):
        send_setting_info_frame = tk.Frame(root)
        send_setting_info_frame.grid(row=row, column=column)

        length_frame = tk.Frame(send_setting_info_frame)
        length_frame.grid(row=0, column=0, padx=PADX, pady=2)
        length_frame_label = tk.Label(
            length_frame,
            text="Length:",   
        )
        length_frame_label.grid(row=0, column=0, padx=PADX, pady=2)
        self.can_data_length = tk.StringVar()
        self.can_data_length_combobox = ttk.Combobox(
            length_frame,
            values=["1", "2", "3", "4", "5", "6", "7", "8"],
            width=4,
            height=8,
            textvariable=self.can_data_length,
        )
        self.can_data_length_combobox.grid(
            row=0,
            column=1,
            padx=2, 
            pady=2
        )
        self.can_data_length_combobox.set("8")
        self.can_data_length_combobox.bind(
            "<<ComboboxSelected>>", self.can_data_length_handler
        )

        id_frame = tk.Frame(send_setting_info_frame)
        id_frame.grid(row=0, column=1)
        id_frame_label = tk.Label(
            id_frame,
            text="ID(hex):",
        )
        id_frame_label.grid(row=0, column=0, padx=PADX, pady=2)
        self.can_data_id = ttk.Entry(id_frame, width=6)
        self.can_data_id.grid(
            row=0,
            column=1,
            padx=2, 
            pady=2
        )
        self.can_data_id.insert(0, "123")
        self.can_data_id.bind(
            "<Button-1>",
            self.can_data_id_handler
        )

        self.create_send_data_info_layer(send_setting_info_frame, row=0, column=2)

    def create_send_data_info_layer(self, root, row, column):
        data_frame = tk.Frame(root)
        data_frame.grid(row=row, column=column)

        data_label = tk.Label(data_frame, text="Data(hex):")
        data_label.grid(row=0, column=0, padx=2, pady=2)

        self.can_send_data1 = tk.Entry(data_frame, width=4)
        self.can_send_data1.grid(
            row=0, column=1, padx=PADX, pady=2
        )
        
        self.can_send_data2 = tk.Entry(data_frame, width=4)
        self.can_send_data2.grid(
            row=0, column=2, padx=PADX, pady=2
        )
        
        self.can_send_data3 = tk.Entry(data_frame, width=4)
        self.can_send_data3.grid(
            row=0, column=3, padx=PADX, pady=2
        )
        
        self.can_send_data4 = tk.Entry(data_frame, width=4)
        self.can_send_data4.grid(
            row=0, column=4, padx=PADX, pady=2
        )
        
        self.can_send_data5 = tk.Entry(data_frame, width=4)
        self.can_send_data5.grid(
            row=0, column=5, padx=PADX, pady=2
        )
        
        self.can_send_data6 = tk.Entry(data_frame, width=4)
        self.can_send_data6.grid(
            row=0, column=6, padx=PADX, pady=2
        )
        
        self.can_send_data7 = tk.Entry(data_frame, width=4)
        self.can_send_data7.grid(
            row=0, column=7, padx=PADX, pady=2
        )
        
        self.can_send_data8 = tk.Entry(data_frame, width=4)
        self.can_send_data8.grid(
            row=0, column=8, padx=PADX, pady=2
        )

        self.can_send_data1.insert(0, "01")
        self.can_send_data2.insert(0, "02")
        self.can_send_data3.insert(0, "03")
        self.can_send_data4.insert(0, "04")
        self.can_send_data5.insert(0, "05")
        self.can_send_data6.insert(0, "06")
        self.can_send_data7.insert(0, "07")
        self.can_send_data8.insert(0, "08")
        self.create_send_button(root, row=0, column=9)

    def create_send_button(self, root, row, column):
        self.can_send_button = tk.Button(
            root,
            text="Send",
            command=self.send_can_button_handler,
            bg="white",
        )
        self.can_send_button.grid(
            row=row, column=column, padx=PADX, pady=PADY
        )
    
    def create_vehicle_control_layer(self, root, row, column):
        self.vehicle_control_frame = tk.LabelFrame(root, text="Vehicle Control")
        self.vehicle_control_frame.grid(row=row, column=column, padx=PADX, pady=PADY, sticky="nsew")

        # initialized vehicle control Layout
        self.vehicle_type_select_handler(None)  

        # self.create_vehicle_control_base_layer(vehicle_control_frame, 0, 0)
        # separator_vertical = ttk.Separator(vehicle_control_frame, orient=tk.VERTICAL)
        # separator_vertical.grid(row=0, column=1, padx=PADX, pady=PADY, sticky="nsew")

        # self.create_vehicle_control_info_layer(vehicle_control_frame, 0, 2)
        # separator_horizontal = ttk.Separator(vehicle_control_frame, orient=tk.HORIZONTAL)
        # separator_horizontal.grid(row=1, column=0, columnspan=3, padx=PADX, pady=PADY, sticky="nsew")
        
        # self.create_vehicle_current_info_layer(vehicle_control_frame, row=2, column=0, columnspan=3)

    def create_vehicle_control_base_layer(self, root, row, column):
        vehicle_control_base_frame = tk.Frame(root)
        vehicle_control_base_frame.grid(row=row, column=column, padx=PADX, pady=PADY, sticky="nsew")

        vehicle_driving_mode_frame = tk.Frame(vehicle_control_base_frame)
        vehicle_driving_mode_frame.grid(row=0, column=0, padx=PADX, pady=PADY, sticky="nsew")
        vehicle_driving_mode_label = tk.Label(
            vehicle_driving_mode_frame,
            text="Driving Mode:"
        )
        vehicle_driving_mode_label.grid(row=0, column=0, padx=PADX, pady=PADY, sticky="nsew")

        # 添加互斥按钮
        self.vehicle_driving_mode = tk.BooleanVar()  # True for Auto, False for Manual
        self.auto_button = tk.Radiobutton(
            vehicle_driving_mode_frame,
            text="Auto",
            variable=self.vehicle_driving_mode,
            value=True,
            command=self.vehicle_driving_mode_handler
        )
        self.auto_button.grid(row=0, column=1, padx=PADX, pady=PADY, sticky="nsew")

        self.manual_button = tk.Radiobutton(
            vehicle_driving_mode_frame,
            text="Manual",
            variable=self.vehicle_driving_mode,
            value=False,
            command=self.vehicle_driving_mode_handler
        )
        self.manual_button.grid(row=0, column=2, padx=PADX, pady=PADY, sticky="nsew")

        # 默认设置为 Manual
        self.vehicle_driving_mode.set(False)
        
        vehicle_gear_frame = tk.Frame(vehicle_control_base_frame)
        vehicle_gear_frame.grid(row=1, column=0, padx=PADX, pady=PADY, sticky="nsew")
        vehicle_gear_label = tk.Label(
            vehicle_gear_frame,
            text="Gear:"
        )
        vehicle_gear_label.grid(row=0, column=0, padx=PADX, pady=PADY, sticky="nsew")
        self.vehicle_gear = tk.StringVar()
        self.vehicle_gear_combobox = ttk.Combobox(
            vehicle_gear_frame,
            values=["P", "R", "N", "D"],
            width=8,
            height=4,
            textvariable=self.vehicle_gear
        )
        self.vehicle_gear_combobox.grid(row=0, column=1, padx=PADX, pady=PADY, sticky="nsew")
        self.vehicle_gear_combobox.set("P")
        self.vehicle_gear_combobox.bind("<<ComboboxSelected>>", self.vehicle_gear_handler)

        separator_vertical1 = ttk.Separator(vehicle_control_base_frame, orient=tk.HORIZONTAL)
        separator_vertical1.grid(row=2, column=0, padx=PADX, pady=PADY, sticky="nsew")

        self.vehicle_throttle = tk.Scale(
            vehicle_control_base_frame,
            from_=0,
            to=100,
            orient=tk.HORIZONTAL,
            label="Throttle(%)",
            width=20,
            length=200,
            command=self.vehicle_throttle_handler
        )
        self.vehicle_throttle.grid(row=3, column=0, padx=PADX, pady=PADY, sticky="nsew")

        separator_vertical2 = ttk.Separator(vehicle_control_base_frame, orient=tk.HORIZONTAL)
        separator_vertical2.grid(row=4, column=0, padx=PADX, pady=PADY, sticky="nsew")

        self.vehicle_brake = tk.Scale(
            vehicle_control_base_frame,
            from_=0,
            to=100,
            orient=tk.HORIZONTAL,
            label="Brake(%)",
            width=20,
            length=200,
            command=self.vehicle_brake_handler
        )
        self.vehicle_brake.grid(row=5, column=0, padx=PADX, pady=PADY, sticky="nsew")

        separator_vertical3 = ttk.Separator(vehicle_control_base_frame, orient=tk.HORIZONTAL)
        separator_vertical3.grid(row=6, column=0, padx=PADX, pady=PADY, sticky="nsew")

        self.vehicle_steering = tk.Scale(
            vehicle_control_base_frame,
            from_=-100,
            to=100,
            orient=tk.HORIZONTAL,
            label="Steering(%)(left|right)",
            width=20,
            length=200,
            command=self.vehicle_steering_handler
        )
        self.vehicle_steering.grid(row=7, column=0, padx=PADX, pady=PADY, sticky="nsew")

        separator_vertical4 = ttk.Separator(vehicle_control_base_frame, orient=tk.HORIZONTAL)
        separator_vertical4.grid(row=8, column=0, padx=PADX, pady=PADY, sticky="nsew")

        vehicle_current_driving_mode_frame = tk.Frame(vehicle_control_base_frame)
        vehicle_current_driving_mode_frame.grid(row=9, column=0, padx=PADX, pady=PADY, sticky="nsew")
        vehicle_current_driving_mode_label = tk.Label(
            vehicle_current_driving_mode_frame,
            text="Driving Mode:"
        )
        vehicle_current_driving_mode_label.grid(row=0, column=0, padx=PADX, pady=PADY, sticky="nsew")
        self.vehicle_current_driving_mode_info = tk.Text(
            vehicle_current_driving_mode_frame,
            height=1,
            width=10
        )
        self.vehicle_current_driving_mode_info.grid(row=0, column=1, padx=PADX, pady=PADY, sticky="nsew")

        vehicle_current_gear_frame = tk.Frame(vehicle_control_base_frame)
        vehicle_current_gear_frame.grid(row=10, column=0, padx=PADX, pady=PADY, sticky="nsew")
        vehicle_current_gear_label = tk.Label(
            vehicle_current_gear_frame,
            text="Gear:"
        )
        vehicle_current_gear_label.grid(row=0, column=0, padx=PADX, pady=PADY, sticky="nsew")
        self.vehicle_current_gear_info = tk.Text(
            vehicle_current_gear_frame,
            height=1,
            width=10
        )
        self.vehicle_current_gear_info.grid(row=0, column=1, padx=PADX, pady=PADY, sticky="nsew")

    def create_vehicle_control_base_layer_LMT(self, root, row, column):
        vehicle_control_base_frame = tk.Frame(root)
        vehicle_control_base_frame.grid(row=row, column=column, padx=PADX, pady=PADY, sticky="nsew")

        # 电机工作模式
        motor_mode_frame = tk.Frame(vehicle_control_base_frame)
        motor_mode_frame.grid(row=0, column=0, padx=PADX, pady=PADY, sticky="nsew")
        motor_mode_label = tk.Label(motor_mode_frame, text="Motor Mode:")
        motor_mode_label.grid(row=0, column=0, padx=PADX, pady=PADY, sticky="nsew")

        # 使用按钮切换模式
        self.motor_mode = tk.StringVar(value="None")  # 默认模式为 None
        self.mode_none_button = tk.Radiobutton(
            motor_mode_frame,
            text="None",
            variable=self.motor_mode,
            value="None",
            command=self.update_motor_mode
        )
        self.mode_none_button.grid(row=0, column=1, padx=PADX, pady=PADY, sticky="nsew")

        self.mode_speed_button = tk.Radiobutton(
            motor_mode_frame,
            text="Speed",
            variable=self.motor_mode,
            value="Speed",
            command=self.update_motor_mode
        )
        self.mode_speed_button.grid(row=0, column=2, padx=PADX, pady=PADY, sticky="nsew")

        self.mode_current_button = tk.Radiobutton(
            motor_mode_frame,
            text="Current",
            variable=self.motor_mode,
            value="Current",
            command=self.update_motor_mode
        )
        self.mode_current_button.grid(row=0, column=3, padx=PADX, pady=PADY, sticky="nsew")

        # 目标速度（改为拉杆形式）
        target_speed_frame = tk.Frame(vehicle_control_base_frame)
        target_speed_frame.grid(row=1, column=0, padx=PADX, pady=PADY, sticky="nsew")
        target_speed_label = tk.Label(target_speed_frame, text="Target Speed (rpm):")
        target_speed_label.grid(row=0, column=0, padx=PADX, pady=PADY, sticky="nsew")
        self.target_speed_scale = tk.Scale(
            target_speed_frame,
            from_=-3000,
            to=3000,
            orient=tk.HORIZONTAL,
            length=200,
            resolution=10,  # 设置步长为10
            label="Speed (rpm)",
            command=self.update_target_speed
        )
        self.target_speed_scale.grid(row=0, column=1, padx=PADX, pady=PADY, sticky="nsew")
        self.target_speed_scale.set(0)  # 默认值为0

        # 目标速度（改为拉杆形式）
        target_steer_angle_frame = tk.Frame(vehicle_control_base_frame)
        target_steer_angle_frame.grid(row=1, column=1, padx=PADX, pady=PADY, sticky="nsew")
        target_steer_angle_label = tk.Label(target_steer_angle_frame, text="Target Speed (rpm):")
        target_steer_angle_label.grid(row=0, column=1, padx=PADX, pady=PADY, sticky="nsew")
        self.target_steer_angle_scale = tk.Scale(
            target_steer_angle_frame,
            from_=-45,
            to=45,
            orient=tk.HORIZONTAL,
            length=200,
            resolution=1,  # 设置步长为1
            label="Steer Angle (deg)",
            command=self.update_target_steer_angle
        )
        self.target_steer_angle_scale.grid(row=0, column=1, padx=PADX, pady=PADY, sticky="nsew")
        self.target_steer_angle_scale.set(0)  # 默认值为0

        # 目标电流（改为拉杆形式）
        target_current_frame = tk.Frame(vehicle_control_base_frame)
        target_current_frame.grid(row=2, column=0, padx=PADX, pady=PADY, sticky="nsew")
        target_current_label = tk.Label(target_current_frame, text="Target Current (A):")
        target_current_label.grid(row=0, column=0, padx=PADX, pady=PADY, sticky="nsew")
        self.target_current_scale = tk.Scale(
            target_current_frame,
            from_=-80,
            to=80,
            orient=tk.HORIZONTAL,
            length=200,
            resolution=1,  # 设置步长为1
            label="Current (A)",
            command=self.update_target_current
        )
        self.target_current_scale.grid(row=0, column=1, padx=PADX, pady=PADY, sticky="nsew")
        self.target_current_scale.set(0)  # 默认值为0

    def update_motor_mode(self):
        mode = self.motor_mode.get()
        self.logger.info(f"Motor mode changed to: {mode}")

    def update_target_steer_angle(self, value):
        angle = int(value)
        self.logger.info(f"Target steer angle set to: {angle} degree")

    def update_target_speed(self, value):
        speed = int(value)
        self.logger.info(f"Target speed set to: {speed} rpm")

    def update_target_current(self, value):
        current = int(value)
        self.logger.info(f"Target current set to: {current} A")

    def create_vehicle_control_info_layer(self, root, row, column):
        vehicle_control_info_frame = tk.Frame(root)
        vehicle_control_info_frame.grid(row=row, column=column, padx=PADX, pady=PADY, sticky="nsew")

        self.vehicle_info_canvas = tk.Canvas(vehicle_control_info_frame, width=550, height=300, bg="white")
        self.vehicle_info_canvas.grid(row=0, column=0, padx=PADX, pady=2, sticky="nsew")

        # 初始化标志变量
        self.vehicle_info_canvas_initialized = True

        # 坐标系
        self.vehicle_info_canvas.create_line(25, 25, 25, 275, width=2, arrow=tk.FIRST)
        self.vehicle_info_canvas.create_line(25, 275, 475, 275, width=2, arrow=tk.LAST)

        for y in range(50, 276, 50):
            self.vehicle_info_canvas.create_line(25, y, 30, y, width=2)
            self.vehicle_info_canvas.create_text(35, y, text=str(150 - y), anchor=tk.W)
        self.vehicle_info_canvas.create_text(485, 275, text="Time", anchor=tk.W)

        # vehicle_info_canvas_label = tk.Label(vehicle_control_info_frame, text="Throttle: Green, Brake: Red, Steering: Blue, Speed(x10): Yellow")
        # vehicle_info_canvas_label.grid(row=1, column=0, padx=PADX, pady=2, sticky="nsew")
        # 添加图例（Legend）
        legend_x = 420  # 图例的起始 x 坐标
        legend_y = 20   # 图例的起始 y 坐标
        line_length = 30  # 线段长度
        line_spacing = 20  # 线段间距
        # Throttle: Green
        self.vehicle_info_canvas.create_line(legend_x, legend_y, legend_x + line_length, legend_y, fill="green", width=2)
        self.vehicle_info_canvas.create_text(legend_x + line_length + 10, legend_y, text="Throttle", anchor=tk.W)
        # Brake: Red
        self.vehicle_info_canvas.create_line(legend_x, legend_y + line_spacing, legend_x + line_length, legend_y + line_spacing, fill="red", width=2)
        self.vehicle_info_canvas.create_text(legend_x + line_length + 10, legend_y + line_spacing, text="Brake", anchor=tk.W)
        # Steering: Blue
        self.vehicle_info_canvas.create_line(legend_x, legend_y + 2 * line_spacing, legend_x + line_length, legend_y + 2 * line_spacing, fill="blue", width=2)
        self.vehicle_info_canvas.create_text(legend_x + line_length + 10, legend_y + 2 * line_spacing, text="Steering", anchor=tk.W)
        # Speed(x10): Yellow
        self.vehicle_info_canvas.create_line(legend_x, legend_y + 3 * line_spacing, legend_x + line_length, legend_y + 3 * line_spacing, fill="#FFA500", width=2)
        self.vehicle_info_canvas.create_text(legend_x + line_length + 10, legend_y + 3 * line_spacing, text="Speed(x10)", anchor=tk.W)

        self.vehicle_history_info = tk.Scale(
            vehicle_control_info_frame, 
            from_=0, 
            to=100, 
            orient="horizontal", 
            length=400, 
            label="History",
            command=self.vehicle_history_info_handler
            )
        self.vehicle_history_info.grid(row=2, column=0, padx=PADX, pady=2, sticky="nsew")
        self.vehicle_history_info.set(100)
        self.update_vehicle_info_canvas()
        

    def create_vehicle_current_info_layer(self, root, row, column, rowspan=1, columnspan=1):
        vehicle_current_info_frame = tk.Frame(root)
        vehicle_current_info_frame.grid(row=row, column=column, rowspan=rowspan, columnspan=columnspan, sticky="nsew")

        throttle_frame = tk.Frame(vehicle_current_info_frame)
        throttle_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        throttle_frame_label = tk.Label(throttle_frame, text="Throttle(%):")
        throttle_frame_label.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.vehicle_current_throttle_info = tk.Text(throttle_frame, height=1, width=5)
        self.vehicle_current_throttle_info.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        brake_frame = tk.Frame(vehicle_current_info_frame)
        brake_frame.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")
        brake_frame_label = tk.Label(brake_frame, text="Brake(%):")
        brake_frame_label.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.vehicle_current_brake_info = tk.Text(brake_frame, height=1, width=5)
        self.vehicle_current_brake_info.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        steering_frame = tk.Frame(vehicle_current_info_frame)
        steering_frame.grid(row=0, column=2, padx=5, pady=5, sticky="nsew")
        steering_frame_label = tk.Label(steering_frame, text="Steering(%):")
        steering_frame_label.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.vehicle_current_steering_info = tk.Text(steering_frame, height=1, width=5)
        self.vehicle_current_steering_info.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        speed_frame = tk.Frame(vehicle_current_info_frame)
        speed_frame.grid(row=0, column=3, padx=5, pady=5, sticky="nsew")
        speed_frame_label = tk.Label(speed_frame, text="Speed(km/h):")
        speed_frame_label.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.vehicle_current_speed_info = tk.Text(speed_frame, height=1, width=5)
        self.vehicle_current_speed_info.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        battery_frame = tk.Frame(vehicle_current_info_frame)
        battery_frame.grid(row=0, column=4, padx=5, pady=5, sticky="nsew")
        battery_frame_label = tk.Label(battery_frame, text="Battery(%):")
        battery_frame_label.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.vehicle_current_battery_info = tk.Text(battery_frame, height=1, width=5)
        self.vehicle_current_battery_info.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        parking_brake = tk.Frame(vehicle_current_info_frame)
        parking_brake.grid(row=0, column=5, padx=5, pady=5, sticky="nsew")
        parking_brake_label = tk.Label(parking_brake, text="EPB:")
        parking_brake_label.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.vehicle_current_parking_info = tk.Text(parking_brake, height=1, width=5)
        self.vehicle_current_parking_info.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")
        self.update_vehicle_info_handler()

    def create_vehicle_current_info_layer_LMT(self, root, row, column, rowspan=1, columnspan=1):
        # 销毁旧的 vehicle_info_canvas 和 vehicle_history_info
        if hasattr(self, "vehicle_info_canvas"):
            self.vehicle_info_canvas.destroy()
            del self.vehicle_info_canvas
            self.vehicle_info_canvas_initialized = False
        if hasattr(self, "vehicle_history_info"):
            self.vehicle_history_info.destroy()
            del self.vehicle_history_info
        
        # 创建电机状态信息的布局
        vehicle_current_info_frame = tk.Frame(root)
        vehicle_current_info_frame.grid(row=row, column=column, rowspan=rowspan, columnspan=columnspan, sticky="nsew")

        # 创建电机 1 的状态信息区域
        motor_1_frame = tk.LabelFrame(vehicle_current_info_frame, text="Motor 1 Status")
        motor_1_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")

        # 当前电流
        current_frame_1 = tk.Frame(motor_1_frame)
        current_frame_1.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        current_label_1 = tk.Label(current_frame_1, text="Current (A):")
        current_label_1.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.vehicle_current_current_info_1 = tk.Text(current_frame_1, height=1, width=10)
        self.vehicle_current_current_info_1.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        # 当前转速
        speed_frame_1 = tk.Frame(motor_1_frame)
        speed_frame_1.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")
        speed_label_1 = tk.Label(speed_frame_1, text="Speed (rpm):")
        speed_label_1.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.vehicle_current_speed_info_1 = tk.Text(speed_frame_1, height=1, width=10)
        self.vehicle_current_speed_info_1.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        # 当前工作模式
        mode_frame_1 = tk.Frame(motor_1_frame)
        mode_frame_1.grid(row=2, column=0, padx=5, pady=5, sticky="nsew")
        mode_label_1 = tk.Label(mode_frame_1, text="Mode:")
        mode_label_1.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.vehicle_current_mode_info_1 = tk.Text(mode_frame_1, height=1, width=10)
        self.vehicle_current_mode_info_1.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        # 遥控器状态
        remote_status_frame_1 = tk.Frame(motor_1_frame)
        remote_status_frame_1.grid(row=3, column=0, padx=5, pady=5, sticky="nsew")
        remote_status_label_1 = tk.Label(remote_status_frame_1, text="Remote Status:")
        remote_status_label_1.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.vehicle_current_remote_status_info_1 = tk.Text(remote_status_frame_1, height=1, width=10)
        self.vehicle_current_remote_status_info_1.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        # 温度
        temperature_frame_1 = tk.Frame(motor_1_frame)
        temperature_frame_1.grid(row=4, column=0, padx=5, pady=5, sticky="nsew")
        temperature_label_1 = tk.Label(temperature_frame_1, text="Temperature (°C):")
        temperature_label_1.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.vehicle_current_temperature_info_1 = tk.Text(temperature_frame_1, height=1, width=10)
        self.vehicle_current_temperature_info_1.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        # 电机脉冲计数
        pulse_count_frame_1 = tk.Frame(motor_1_frame)
        pulse_count_frame_1.grid(row=5, column=0, padx=5, pady=5, sticky="nsew")
        pulse_count_label_1 = tk.Label(pulse_count_frame_1, text="Pulse Count:")
        pulse_count_label_1.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.vehicle_current_pulse_count_info_1 = tk.Text(pulse_count_frame_1, height=1, width=10)
        self.vehicle_current_pulse_count_info_1.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        # 创建电机 2 的状态信息区域
        motor_2_frame = tk.LabelFrame(vehicle_current_info_frame, text="Motor 2 Status")
        motor_2_frame.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        # 当前电流
        current_frame_2 = tk.Frame(motor_2_frame)
        current_frame_2.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        current_label_2 = tk.Label(current_frame_2, text="Current (A):")
        current_label_2.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.vehicle_current_current_info_2 = tk.Text(current_frame_2, height=1, width=10)
        self.vehicle_current_current_info_2.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        # 当前转速
        speed_frame_2 = tk.Frame(motor_2_frame)
        speed_frame_2.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")
        speed_label_2 = tk.Label(speed_frame_2, text="Speed (rpm):")
        speed_label_2.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.vehicle_current_speed_info_2 = tk.Text(speed_frame_2, height=1, width=10)
        self.vehicle_current_speed_info_2.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        # 当前工作模式
        mode_frame_2 = tk.Frame(motor_2_frame)
        mode_frame_2.grid(row=2, column=0, padx=5, pady=5, sticky="nsew")
        mode_label_2 = tk.Label(mode_frame_2, text="Mode:")
        mode_label_2.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.vehicle_current_mode_info_2 = tk.Text(mode_frame_2, height=1, width=10)
        self.vehicle_current_mode_info_2.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        # 遥控器状态
        remote_status_frame_2 = tk.Frame(motor_2_frame)
        remote_status_frame_2.grid(row=3, column=0, padx=5, pady=5, sticky="nsew")
        remote_status_label_2 = tk.Label(remote_status_frame_2, text="Remote Status:")
        remote_status_label_2.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.vehicle_current_remote_status_info_2 = tk.Text(remote_status_frame_2, height=1, width=10)
        self.vehicle_current_remote_status_info_2.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        # 温度
        temperature_frame_2 = tk.Frame(motor_2_frame)
        temperature_frame_2.grid(row=4, column=0, padx=5, pady=5, sticky="nsew")
        temperature_label_2 = tk.Label(temperature_frame_2, text="Temperature (°C):")
        temperature_label_2.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.vehicle_current_temperature_info_2 = tk.Text(temperature_frame_2, height=1, width=10)
        self.vehicle_current_temperature_info_2.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        # 电机脉冲计数
        pulse_count_frame_2 = tk.Frame(motor_2_frame)
        pulse_count_frame_2.grid(row=5, column=0, padx=5, pady=5, sticky="nsew")
        pulse_count_label_2 = tk.Label(pulse_count_frame_2, text="Pulse Count:")
        pulse_count_label_2.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.vehicle_current_pulse_count_info_2 = tk.Text(pulse_count_frame_2, height=1, width=10)
        self.vehicle_current_pulse_count_info_2.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        # update vehicle status layout
        self.update_vehicle_info_handler_LMT()
        

    def create_send_info_layer(self, root, row=0, column=0):
        send_info_frame = tk.LabelFrame(root, text="Send Monitor")
        send_info_frame.grid(row=row, column=column, padx=5, pady=5, sticky="nsew")
        send_info_frame.grid_rowconfigure(0, weight=1)
        send_info_frame.grid_columnconfigure(0, weight=1)
        send_info_frame.grid_columnconfigure(1, weight=0)

        self.can_send_info = tk.Text(
            send_info_frame, height=10, width=100, font="Consoles 8"
        )
        self.can_send_info.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        scrollbar = tk.Scrollbar(send_info_frame, command=self.can_send_info.yview)
        scrollbar.grid(row=0, column=1, sticky="nsew")
        self.can_send_info.config(yscrollcommand=scrollbar.set)
        # self.update_send_info_handle()

    def create_recv_info_layer(self, root, row=0, column=0):
        receive_info_frame = tk.LabelFrame(root, text="Receive Monitor")
        receive_info_frame.grid(row=row, column=column, padx=5, pady=5, sticky="nsew")
        receive_info_frame.grid_rowconfigure(0, weight=1)
        receive_info_frame.grid_columnconfigure(0, weight=1)
        receive_info_frame.grid_columnconfigure(1, weight=0)
        
        self.can_recv_info = tk.Text(
            receive_info_frame, height=10, width=100, font="Consoles 8"
        )
        self.can_recv_info.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        scrollbar = tk.Scrollbar(receive_info_frame, command=self.can_recv_info.yview)
        scrollbar.grid(row=0, column=1, sticky="nsew")
        self.can_recv_info.config(yscrollcommand=scrollbar.set)
        # self.update_recv_info_handle()

    def can_device_combobox_select_handler(self, _event):
        self.logger.debug(f"select can device {self.can_device.get()}")

    def print_status_log(self, log, level="info"):
        # 定义不同级别的颜色
        color_map = {
            "info": "black",
            "warning": "orange",
            "error": "red",
            "debug": "blue",
        }
        
        # 获取对应级别的颜色，默认为黑色
        color = color_map.get(level.lower(), "black")
        
        # 插入文本并应用颜色标签
        self.status_log_text.insert(tk.END, f"{log}\n", level)
        self.status_log_text.tag_config(level, foreground=color)
        self.status_log_text.see(tk.END)

    # 只有连接后才可做别的操作，否则提示先连接设备
    # 这是总开关
         
    def create_base_config_layer(self, root, row=0, column=0):
        CONNECT_DEVICE = "Connect Device"
        connect_device_frame = tk.LabelFrame(root, text=CONNECT_DEVICE)
        connect_device_frame.grid(row=row, column=column, padx=PADX, pady=PADY, sticky="nsew")

        connect_device_frame.grid_rowconfigure(0, weight=1)
        connect_device_frame.grid_columnconfigure(0, weight=1)

        self.can_device = tk.StringVar()
        self.can_device_combobox = ttk.Combobox(
            connect_device_frame,
            values=["PCAN_USBBUS1", "PCAN_USBBUS2"],
            width=20,
            textvariable=self.can_device,
            state="readonly"
        )
        self.can_device_combobox.grid(row=0, column=0, padx=PADX, pady=PADY, sticky="ew")
        self.can_device_combobox.bind(
            "<<ComboboxSelected>>", self.can_device_combobox_select_handler
        )
        self.can_device_combobox.set("PCAN_USBBUS1")  # Set default to first USB bus

        self.can_connect_button = tk.Button(
            connect_device_frame,
            text="Connect",
            command=self.can_connect_button_handler,
            width=10,
            bg="white"
        )
        self.can_connect_button.grid(row=1, column=0, padx=PADX, pady=PADY, sticky="nsew")

        self.can_start_button = tk.Button(
            connect_device_frame,
            text="Receive",
            command=self.can_start_button_handler,
            width=10,
            bg="white"
        )
        self.can_start_button.grid(row=2, column=0, padx=PADX, pady=PADY, sticky="nsew")

    def can_start_button_handler(self):
        if not self.can_connect_status:
            curr_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.print_status_log(f"{curr_time} Please connect device first", level="error")
            self.logger.debug(f"please connect device first")
            return
        self.can_recv_status = not self.can_recv_status
        if self.can_recv_status:
            self.can_start_button.config(text="Stop Recv", bg="red")
            self.can_start_button.update()
        else:
            self.can_start_button.config(text="Receive", bg="white")
            self.can_start_button.update()

    def send_can_button_handler(self):
        if not self.can_connect_status:
            curr_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.print_status_log(f"{curr_time} Please connect device first", level="error")
            self.logger.debug(f"please connect device first")
            return

        self.can_send_status = not self.can_send_status
        if self.can_send_status:
            self.can_send_button.config(text="Stop", bg="red")
            self.can_send_button.update()
        else:
            self.can_send_button.config(text="Send", bg="white")
            self.can_send_button.update()

    def can_extend_mode_handler(self):
        is_selected = self.can_extend_mode_check_button.getvar(
            self.can_extend_mode_check_button["variable"]
        )
        self.logger.debug(f"extern mode: {is_selected}")

    def can_increase_id_handler(self):
        is_select = self.can_increase_id_check_button.getvar(
            self.can_increase_id_check_button["variable"]
        )
        self.logger.debug(f"increase id: {is_select}")

    def can_interval_handler(self, _event):
        self.logger.debug(f"can interval: {self.can_interval.get()}")
    
    def can_times_handler(self, _event):
        self.logger.debug(f"can send times: {self.can_times.get()}")
        self.logger.debug(f"can send times: {self.can_times.get()}")

    def mode_select_handler(self, _event):
        self.logger.debug(f"mode: {self.mode.get()}")

    def can_data_length_handler(self, _event):
        self.logger.debug(f"can data length: {self.can_data_length.get()}")

    def can_data_id_handler(self, _event):
        self.logger.debug(f"can data length: {self.can_data_id.get()}")
    
    def vehicle_driving_mode_handler(self):
        current_driving_mode = "Auto" if self.vehicle_driving_mode.get() else "Manual"
        self.logger.debug(f"Driving mode changed to: {current_driving_mode}")
        self.vehicle_current_driving_mode_info.delete("1.0", "end")
        self.vehicle_current_driving_mode_info.insert("end", current_driving_mode)

    def vehicle_gear_handler(self, _event):
        self.logger.debug(f"gear: {self.vehicle_gear.get()}")

    def vehicle_throttle_handler(self, _event):
        self.logger.debug(f"throttle: {self.vehicle_throttle.get()}")

    def vehicle_brake_handler(self, _event):
        self.logger.debug(f"brake: {self.vehicle_brake.get()}")

    def vehicle_steering_handler(self, _event):
        self.logger.debug(f"steering: {self.vehicle_steering.get()}")

    def vehicle_history_info_handler(self, _event):
        self.logger.debug(f"history: {self.vehicle_history_info.get()}")

    def update_vehicle_info_canvas(self):
        if not self.vehicle_info_canvas_initialized:
            self.logger.warning("vehicle_info_canvas is not initialized or has been destroyed. Skipping update.")
            return
        if not hasattr(self, "vehicle_info_canvas"):
            self.logger.warning("vehicle_info_canvas attribute does not exist. Skipping update.")
            return
        if not hasattr(self, "vehicle_history_info"):
            self.logger.warning("vehicle_history_info is not initialized or has been destroyed. Skipping update.")
            return

        MAX_POINTS = 100
        # last_vehicle_status = VehicleStatus()
        throttle_deque = deque(maxlen=MAX_POINTS)
        brake_deque = deque(maxlen=MAX_POINTS)
        steering_deque = deque(maxlen=MAX_POINTS)
        speed_deque = deque(maxlen=MAX_POINTS)
        
        def draw_curve():
            # update vehicle history info
            vehicle_status = self.can_report_handler.get_vehicle_status()
            if not vehicle_status is None:
                throttle_deque.append(int(vehicle_status.throttle))
                brake_deque.append(int(vehicle_status.brake))
                steering_deque.append(int(vehicle_status.steering))
                speed_deque.append(int(vehicle_status.speed))

            if not hasattr(self, "vehicle_info_canvas"):
                self.logger.warning("vehicle_info_canvas attribute does not exist. Skipping draw curve.")
                return
        
            self.vehicle_info_canvas.delete("curve")
            
            if len(throttle_deque) > MAX_POINTS:
                self.vehicle_history_info.config(from_=0, to=len(throttle_deque) - 1)
                self.vehicle_history_info.set(len(throttle_deque) - 1)

            start_index = max(0, len(throttle_deque) - MAX_POINTS)
            end_index = start_index + self.vehicle_history_info.get()                  
            current_throttle_list = list(throttle_deque)[start_index:end_index+1]
            current_brake_list = list(brake_deque)[start_index:end_index+1]
            current_steering_list = list(steering_deque)[start_index:end_index+1]
            current_speed_list = list(speed_deque)[start_index:end_index+1]

            if len(current_throttle_list) > 2:
                points = []
                for i, value in enumerate(current_throttle_list):
                    x = 25 + (i / (len(current_throttle_list) - 1)) * 450
                    # y = 275 - (value / 100) * 250
                    y = 150 - value
                    points.append([x, y])
                self.vehicle_info_canvas.create_line(
                    points,
                    fill="green",
                    width=2,
                    smooth=True,
                    tags="curve"
                )

            if len(current_brake_list) > 2:
                points = []
                for i, value in enumerate(current_brake_list):
                    x = 25 + (i / (len(current_brake_list) - 1)) * 450
                    # y = 275 - (value / 100) * 250
                    y = 150 + value
                    points.append([x, y])
                self.vehicle_info_canvas.create_line(
                    points,
                    fill="red",
                    width=2,
                    smooth=True,
                    tags="curve"
                )

            if len(current_steering_list) > 2:
                points = []
                for i, value in enumerate(current_steering_list):
                    x = 25 + (i / (len(current_steering_list) - 1)) * 450
                    # y = 275 - (value / 100) * 250
                    y = 150 - value
                    points.append([x, y])
                self.vehicle_info_canvas.create_line(
                    points,
                    fill="blue",
                    width=2,
                    smooth=True,
                    tags="curve"
                )

            if len(current_speed_list) > 2:
                points = []
                for i, value in enumerate(current_speed_list):
                    x = 25 + (i / (len(current_speed_list) - 1)) * 450
                    # y = 275 - (value / 100) * 250
                    y = 150 - value
                    points.append([x, y])
                self.vehicle_info_canvas.create_line(
                    points,
                    fill="#FFA500",
                    width=2,
                    smooth=True,
                    tags="curve"
                )
            self.root.after(100, draw_curve)

        draw_curve()

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root, log_level=logging.INFO)
    app.spin()
