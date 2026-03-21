#!/usr/bin/env python3

"""
穿越机地面站软件 V1.2 适配匿名科创ANO上位机协议
功能：
1. 实时姿态角显示（Roll, Pitch, Yaw）
2. 3D姿态可视化（无人机模型实时旋转，优化精致模型）
3. 遥控通道监控（0-8通道）
4. 数据记录和CSV导出
5. 传感器原始数据显示+波形显示
6. IMU原始数据波形实时显示（加速度计+陀螺仪）
7. 串口稳定通信+异常重连
8. 模拟数据模式（无硬件也可测试）
9. 完整适配匿名科创ANO飞控通信协议（帧头、校验、功能码）
"""

import sys
import time
import struct
import threading
import csv
from datetime import datetime
import random

import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# 设置中文字体，解决中文显示问题
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False
# 解决matplotlib在tkinter中卡顿问题
plt.rcParams['figure.autolayout'] = True


class DroneGroundStation:
    def __init__(self):
        # 串口配置
        self.serial_port = None
        self.baud_rate = 115200
        self.is_connected = False
        self.simulate_mode = False  # 模拟数据模式，无硬件时开启测试

        # 数据缓冲区
        self.euler_angles = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.rc_channels = [1500] * 9  # 0-8通道，遥控器中位默认1500
        self.imu_raw = {'acc': [0, 0, 0], 'gyro': [0, 0, 0]}
        self.quaternion = [1.0, 0.0, 0.0, 0.0]  # 四元数
        self.shock_status = 0  # 震动状态
        # 传感器数据（气压计、罗盘）
        self.sensor_data = {
            'mag': [0, 0, 0],     # 磁罗盘 X, Y, Z
            'alt_bar': 0,          # 气压高度 (cm)
            'temp': 0,            # 温度 (°C)
            'bar_sta': 0,         # 气压计状态
            'mag_sta': 0          # 罗盘状态
        }

        # 数据记录
        self.data_log = []
        self.is_recording = False
        self.record_start_time = None

        # 历史数据（用于绘图）
        self.max_history = 200
        self.time_history = []
        self.roll_history = []
        self.pitch_history = []
        self.yaw_history = []
        # IMU原始数据历史
        self.acc_x_history = []
        self.acc_y_history = []
        self.acc_z_history = []
        self.gyro_x_history = []
        self.gyro_y_history = []
        self.gyro_z_history = []
        self.start_plot_time = time.time()

        # 线程控制
        self.receive_thread = None
        self.is_thread_running = False

        # 启动GUI
        self.setup_gui()
        self.log_message("地面站启动完成，已适配匿名科创ANO通信协议")
        self.log_message("3D无人机模型已优化，支持完整机身、机臂、螺旋桨可视化")
        self.log_message("已修正俯仰、横滚旋转方向，贴合实际操控直觉，抬头显示抬头、横滚方向匹配操控")
        self.log_message("V1.2新增：IMU原始数据实时波形显示")

    def setup_gui(self):
        """设置图形用户界面，优化布局和控件样式"""
        self.root = tk.Tk()
        self.root.title("穿越机地面站 V1.2（ANO协议版）")
        self.root.geometry("1450x920")
        # 窗口最小大小限制
        self.root.minsize(1200, 800)

        # 设置中文字体
        default_font = ('Microsoft YaHei', 9)
        self.root.option_add('*Font', default_font)

        # 主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # 顶部控制栏
        control_frame = ttk.LabelFrame(main_frame, text="串口控制 & 操作")
        control_frame.pack(fill=tk.X, pady=(0, 10), padx=5, ipady=5)

        # 串口配置区
        ttk.Label(control_frame, text="串口号:").grid(row=0, column=0, padx=5, pady=5)
        self.port_entry = ttk.Entry(control_frame, width=15)
        self.port_entry.insert(0, "COM8")
        self.port_entry.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(control_frame, text="波特率:").grid(row=0, column=2, padx=5, pady=5)
        self.baud_entry = ttk.Entry(control_frame, width=10)
        self.baud_entry.insert(0, "115200")
        self.baud_entry.grid(row=0, column=3, padx=5, pady=5)

        # 控制按钮区
        self.connect_btn = ttk.Button(control_frame, text="连接串口", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=4, padx=8, pady=5)

        self.sim_btn = ttk.Button(control_frame, text="开启模拟", command=self.toggle_simulate)
        self.sim_btn.grid(row=0, column=5, padx=8, pady=5)

        self.record_btn = ttk.Button(control_frame, text="开始记录", command=self.toggle_recording)
        self.record_btn.grid(row=0, column=6, padx=8, pady=5)

        self.clear_btn = ttk.Button(control_frame, text="清空日志", command=self.clear_log)
        self.clear_btn.grid(row=0, column=7, padx=8, pady=5)

        # 状态标签
        self.status_label = ttk.Label(control_frame, text="状态：未连接", foreground="red")
        self.status_label.grid(row=0, column=8, padx=15, pady=5)

        # 主要内容区域
        content_frame = ttk.Frame(main_frame)
        content_frame.pack(fill=tk.BOTH, expand=True)

        # 左侧：数据显示面板
        left_frame = ttk.Frame(content_frame, width=380)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8), pady=5)
        left_frame.pack_propagate(False)

        # 姿态角数值显示
        attitude_frame = ttk.LabelFrame(left_frame, text="实时姿态角 (°)")
        attitude_frame.pack(fill=tk.X, pady=(0, 10), ipady=5)

        self.attitude_labels = {}
        angle_config = [
            ('roll', '横滚', '#FF6B6B'),
            ('pitch', '俯仰', '#4ECDC4'),
            ('yaw', '偏航', '#45B7D1')
        ]
        for i, (angle, name, color) in enumerate(angle_config):
            ttk.Label(attitude_frame, text=f"{name}({angle.upper()}):").grid(row=i, column=0, padx=8, pady=6, sticky='e')
            self.attitude_labels[angle] = ttk.Label(attitude_frame, text="0.00", font=('Courier New', 13, 'bold'), foreground=color)
            self.attitude_labels[angle].grid(row=i, column=1, padx=8, pady=6, sticky='w')

        # 遥控通道显示
        rc_frame = ttk.LabelFrame(left_frame, text="遥控通道值 (CH0-CH8)")
        rc_frame.pack(fill=tk.X, pady=(0, 10), ipady=5)

        self.rc_labels = []
        for i in range(9):
            row = i // 3
            col = (i % 3) * 2
            ttk.Label(rc_frame, text=f"CH{i}:").grid(row=row, column=col, padx=5, pady=4, sticky='e')
            label = ttk.Label(rc_frame, text="1500", font=('Courier New', 10))
            label.grid(row=row, column=col + 1, padx=5, pady=4, sticky='w')
            self.rc_labels.append(label)

        # 传感器数据
        sensor_frame = ttk.LabelFrame(left_frame, text="IMU传感器原始数据")
        sensor_frame.pack(fill=tk.X, pady=(0, 10), ipady=5)

        # 加速度计
        ttk.Label(sensor_frame, text="加速度计 (g):").grid(row=0, column=0, columnspan=6, sticky='w', padx=5, pady=3)
        self.acc_labels = []
        for i, axis in enumerate(['X', 'Y', 'Z']):
            ttk.Label(sensor_frame, text=f"A{axis}:").grid(row=1, column=i * 2, padx=4, sticky='e')
            label = ttk.Label(sensor_frame, text="0.000", font=('Courier New', 9))
            label.grid(row=1, column=i * 2 + 1, padx=4, sticky='w')
            self.acc_labels.append(label)

        # 陀螺仪
        ttk.Label(sensor_frame, text="陀螺仪 (°/s):").grid(row=2, column=0, columnspan=6, sticky='w', padx=5, pady=(6, 3))
        self.gyro_labels = []
        for i, axis in enumerate(['X', 'Y', 'Z']):
            ttk.Label(sensor_frame, text=f"G{axis}:").grid(row=3, column=i * 2, padx=4, sticky='e')
            label = ttk.Label(sensor_frame, text="0.000", font=('Courier New', 9))
            label.grid(row=3, column=i * 2 + 1, padx=4, sticky='w')
            self.gyro_labels.append(label)

        # 气压计和温度
        baro_frame = ttk.LabelFrame(left_frame, text="气压计 & 温度")
        baro_frame.pack(fill=tk.X, pady=(0, 10), ipady=5)

        ttk.Label(baro_frame, text="气压高度:").grid(row=0, column=0, padx=5, pady=4, sticky='e')
        self.alt_label = ttk.Label(baro_frame, text="0 cm", font=('Courier New', 11, 'bold'))
        self.alt_label.grid(row=0, column=1, padx=5, pady=4, sticky='w')

        ttk.Label(baro_frame, text="温度:").grid(row=1, column=0, padx=5, pady=4, sticky='e')
        self.temp_label = ttk.Label(baro_frame, text="0.0 °C", font=('Courier New', 11, 'bold'))
        self.temp_label.grid(row=1, column=1, padx=5, pady=4, sticky='w')

        # 右侧：图形显示区域
        right_frame = ttk.Frame(content_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, pady=5)

        # 3D姿态显示
        fig_3d = plt.figure(figsize=(7, 4.5))
        self.ax_3d = fig_3d.add_subplot(111, projection='3d')
        self.init_3d_plot()
        self.canvas_3d = FigureCanvasTkAgg(fig_3d, right_frame)
        self.canvas_3d.get_tk_widget().pack(fill=tk.BOTH, expand=True, pady=(0, 6))

        # 姿态角实时曲线
        fig_2d = plt.figure(figsize=(7, 2.8))
        self.ax_2d = fig_2d.add_subplot(111)
        self.init_2d_plot()
        self.canvas_2d = FigureCanvasTkAgg(fig_2d, right_frame)
        self.canvas_2d.get_tk_widget().pack(fill=tk.BOTH, expand=True, pady=(0, 6))

        # IMU原始数据波形曲线（加速度计+陀螺仪）
        fig_imu = plt.figure(figsize=(7, 2.8))
        self.ax_imu = fig_imu.add_subplot(111)
        self.init_imu_plot()
        self.canvas_imu = FigureCanvasTkAgg(fig_imu, right_frame)
        self.canvas_imu.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # 底部日志区域
        log_frame = ttk.LabelFrame(main_frame, text="系统日志")
        log_frame.pack(fill=tk.BOTH, pady=(10, 0), padx=5)

        self.log_text = scrolledtext.ScrolledText(log_frame, height=7, wrap=tk.WORD, bg="#F8F9FA")
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        # 日志框默认只读
        self.log_text.config(state=tk.DISABLED)

        # 启动界面刷新定时器
        self.root.after(50, self.update_display)

    def init_3d_plot(self):
        """初始化3D姿态可视化画布，优化画布参数适配精致无人机模型"""
        self.ax_3d.clear()
        self.ax_3d.set_xlabel('X轴（横滚）', labelpad=8, fontsize=9)
        self.ax_3d.set_ylabel('Y轴（偏航）', labelpad=8, fontsize=9)
        self.ax_3d.set_zlabel('Z轴（俯仰/高度）', labelpad=8, fontsize=9)
        self.ax_3d.set_title('穿越机3D姿态实时显示（优化模型+直觉姿态方向）', fontsize=11, pad=12, weight='bold')
        # 调整坐标轴范围，适配优化后模型尺寸
        self.ax_3d.set_xlim(-2.5, 2.5)
        self.ax_3d.set_ylim(-2.5, 2.5)
        self.ax_3d.set_zlim(0, 3.5)
        # 优化视角，更直观查看无人机姿态，关闭自动旋转
        self.ax_3d.view_init(elev=30, azim=50)
        # 关闭坐标轴网格，提升视觉整洁度
        self.ax_3d.grid(False)

    def init_2d_plot(self):
        """初始化2D四元数曲线"""
        self.ax_2d.clear()
        self.ax_2d.set_title('四元数实时变化曲线', fontsize=11, pad=10)
        self.ax_2d.set_xlabel('时间 (s)')
        self.ax_2d.set_ylabel('四元数值')
        self.ax_2d.grid(True, alpha=0.3)
        self.ax_2d.set_ylim(-1.2, 1.2)
        # 初始化四条曲线
        self.line_q0, = self.ax_2d.plot([], [], '#E74C3C', label='q0', linewidth=1.2)
        self.line_q1, = self.ax_2d.plot([], [], '#27AE60', label='q1', linewidth=1.2)
        self.line_q2, = self.ax_2d.plot([], [], '#3498DB', label='q2', linewidth=1.2)
        self.line_q3, = self.ax_2d.plot([], [], '#9B59B6', label='q3', linewidth=1.2)
        self.ax_2d.legend(loc='upper right', fontsize=9)

    def init_imu_plot(self):
        """初始化IMU原始数据波形曲线"""
        self.ax_imu.clear()
        self.ax_imu.set_title('IMU原始数据波形', fontsize=11, pad=10)
        self.ax_imu.set_xlabel('时间 (s)')
        self.ax_imu.set_ylabel('原始值')
        self.ax_imu.grid(True, alpha=0.3)
        # 加速度计曲线（g）
        self.line_acc_x, = self.ax_imu.plot([], [], '#E74C3C', label='AccX(g)', linewidth=1.0)
        self.line_acc_y, = self.ax_imu.plot([], [], '#27AE60', label='AccY(g)', linewidth=1.0)
        self.line_acc_z, = self.ax_imu.plot([], [], '#3498DB', label='AccZ(g)', linewidth=1.0)
        # 陀螺仪曲线（°/s）
        self.line_gyro_x, = self.ax_imu.plot([], [], '#9B59B6', label='GyroX(°/s)', linewidth=1.0, linestyle='--')
        self.line_gyro_y, = self.ax_imu.plot([], [], '#F39C12', label='GyroY(°/s)', linewidth=1.0, linestyle='--')
        self.line_gyro_z, = self.ax_imu.plot([], [], '#1ABC9C', label='GyroZ(°/s)', linewidth=1.0, linestyle='--')
        self.ax_imu.legend(loc='upper right', fontsize=8, ncol=2)

    def log_message(self, msg):
        """日志输出函数，带时间戳"""
        current_time = datetime.now().strftime("%H:%M:%S")
        log_str = f"[{current_time}] {msg}\n"
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, log_str)
        # 自动滚动到最新日志
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def clear_log(self):
        """清空日志区域"""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state=tk.DISABLED)
        self.log_message("日志已清空")

    def update_status(self, text, color="red"):
        """更新连接状态标签"""
        self.status_label.config(text=f"状态：{text}", foreground=color)

    def toggle_simulate(self):
        """切换模拟数据模式，无硬件时测试界面"""
        if self.is_connected:
            messagebox.showwarning("提示", "请先断开串口连接，再切换模拟模式")
            return
        self.simulate_mode = not self.simulate_mode
        if self.simulate_mode:
            self.sim_btn.config(text="关闭模拟")
            self.is_connected = True
            self.is_thread_running = True
            self.update_status("模拟数据模式", "green")
            self.log_message("已开启模拟数据模式，自动生成ANO协议格式数据，姿态方向已优化贴合直觉")
            # 启动模拟数据线程
            self.receive_thread = threading.Thread(target=self.simulate_data_thread, daemon=True)
            self.receive_thread.start()
        else:
            self.sim_btn.config(text="开启模拟")
            self.is_connected = False
            self.is_thread_running = False
            self.update_status("未连接", "red")
            self.log_message("已关闭模拟数据模式")

    def toggle_connection(self):
        """切换串口连接/断开状态"""
        if self.simulate_mode:
            messagebox.showwarning("提示", "请先关闭模拟模式，再连接串口")
            return
        if self.is_connected:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        """连接串口，初始化通信"""
        try:
            port = self.port_entry.get().strip()
            baud = int(self.baud_entry.get().strip())
            if not port:
                messagebox.showerror("错误", "请输入有效的串口号，如COM3、COM8")
                return

            self.serial_port = serial.Serial(port, baud, timeout=0.05)
            self.is_connected = True
            self.is_thread_running = True
            self.connect_btn.config(text="断开串口")
            self.update_status(f"已连接 {port}@{baud}", "green")
            self.log_message(f"串口连接成功：{port}，波特率{baud}")
            self.log_message("等待接收ANO协议飞控数据，姿态旋转方向已修正，贴合操控直觉")

            # 启动串口数据接收线程
            self.receive_thread = threading.Thread(target=self.receive_data_thread, daemon=True)
            self.receive_thread.start()

        except serial.SerialException as e:
            self.log_message(f"串口连接失败：{str(e)}")
            messagebox.showerror("串口错误", f"连接失败：{str(e)}\n请检查串口号、波特率，或是否被占用")
        except ValueError:
            self.log_message("波特率输入错误，请输入纯数字")
            messagebox.showerror("输入错误", "波特率必须为纯数字")
        except Exception as e:
            self.log_message(f"未知错误：{str(e)}")

    def disconnect_serial(self):
        """断开串口连接，清理线程"""
        if not self.is_connected:
            return
        try:
            self.is_thread_running = False
            self.is_connected = False
            # 关闭串口
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            # 等待线程退出
            time.sleep(0.1)
            self.connect_btn.config(text="连接串口")
            self.update_status("未连接", "red")
            self.log_message("串口已断开")
        except Exception as e:
            self.log_message(f"断开串口出错：{str(e)}")

    # ====================== 匿名科创ANO协议核心解析模块 ======================
    def ano_verify_checksum(self, frame):
        """
        ANO协议校验和验证
        :param frame: 完整ANO数据帧
        :return: 校验通过返回True，失败返回False
        """
        if len(frame) < 6:
            return False
        data_len = frame[3]
        # 计算范围：帧头+地址+功能码+数据段
        calc_sum = 0
        calc_add = 0
        for i in range(data_len + 4):
            calc_sum += frame[i]
            calc_add += calc_sum
        # 取低8位
        calc_sum &= 0xFF
        calc_add &= 0xFF
        # 对比帧尾校验位
        return (calc_sum == frame[-2]) and (calc_add == frame[-1])

    def euler_angles_from_quaternion(self):
        """从四元数转换为欧拉角（避免万向节锁问题）"""
        q0, q1, q2, q3 = self.quaternion

        # 归一化四元数
        norm = np.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        if norm < 0.001:
            return
        q0, q1, q2, q3 = q0/norm, q1/norm, q2/norm, q3/norm

        # 横滚角 (Roll) - 交换q1,q2
        roll = np.arctan2(2.0*(q0*q2 + q1*q3), 1.0 - 2.0*(q2*q2 + q1*q1))
        # 俯仰角 (Pitch) - 交换q1,q2
        sinp = 2.0*(q0*q1 - q3*q2)
        if np.abs(sinp) >= 1:
            pitch = np.copysign(np.pi/2, sinp)
        else:
            pitch = np.arcsin(sinp)
        # 偏航角 (Yaw)
        yaw = np.arctan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3))

        # 转换为角度
        self.euler_angles['roll'] = np.degrees(roll)
        self.euler_angles['pitch'] = -np.degrees(pitch)
        self.euler_angles['yaw'] = np.degrees(yaw)

    def parse_ano_frame(self, frame):
        """
        解析ANO协议完整数据帧，适配你提供的飞控发送函数
        功能码对应：0x01-IMU原始数据  0x03-欧拉角  0x04-四元数
        """
        try:
            func_code = frame[2]  # 功能码
            data_len = frame[3]   # 数据长度

            # 功能码0x03：欧拉角（俯仰、横滚、偏航）
            # 根据您的代码: uint16_t A_int = (uint16_t)(A * 100);
            # 数据顺序：A(俯仰pitch), B(横滚roll), C(偏航yaw)
            if func_code == 0x03 and data_len == 0x07:
                # 解析姿态角数据（3个int16，每个代表角度*100）
                # 注意：您的代码中A是俯仰，B是横滚，C是偏航
                pitch_raw = struct.unpack('<h', frame[4:6])[0]  # A - 俯仰角
                roll_raw = struct.unpack('<h', frame[6:8])[0]   # B - 横滚角
                yaw_raw = struct.unpack('<h', frame[8:10])[0]   # C - 偏航角

                # 转换为浮点角度值（除以100）
                self.euler_angles['pitch'] = pitch_raw / 100.0
                self.euler_angles['roll'] = roll_raw / 100.0
                self.euler_angles['yaw'] = yaw_raw / 100.0

            # 功能码0x01：IMU原始数据（加速度、陀螺仪、震动状态）
            elif func_code == 0x01 and data_len == 0x0D:
                # 解析加速度计int16
                ax = struct.unpack('<h', frame[4:6])[0]
                ay = struct.unpack('<h', frame[6:8])[0]
                az = struct.unpack('<h', frame[8:10])[0]
                # 解析陀螺仪int16
                gx = struct.unpack('<h', frame[10:12])[0]
                gy = struct.unpack('<h', frame[12:14])[0]
                gz = struct.unpack('<h', frame[14:16])[0]
                # 震动状态
                self.shock_status = frame[16]
                # 单位转换（适配常规飞控量程）
                self.imu_raw['acc'] = [ax / 1000.0, ay / 1000.0, az / 1000.0]
                self.imu_raw['gyro'] = [gx / 10.0, gy / 10.0, gz / 10.0]

            # 功能码0x04：四元数数据
            elif func_code == 0x04 and data_len == 0x09:
                # 四元数放大10000倍发送，需还原
                v0 = struct.unpack('<h', frame[4:6])[0] / 10000.0
                v1 = struct.unpack('<h', frame[6:8])[0] / 10000.0
                v2 = struct.unpack('<h', frame[8:10])[0] / 10000.0
                v3 = struct.unpack('<h', frame[10:12])[0] / 10000.0
                # 归一化四元数
                norm = np.sqrt(v0*v0 + v1*v1 + v2*v2 + v3*v3)
                if norm > 0.001:
                    v0, v1, v2, v3 = v0/norm, v1/norm, v2/norm, v3/norm
                self.quaternion = [v0, v1, v2, v3]
                # 从四元数转换为欧拉角（避免万向节锁）
                self.euler_angles_from_quaternion()

            # 功能码0x02：传感器数据（罗盘、气压、温度）
            elif func_code == 0x02 and data_len == 0x0E:
                # 解析磁罗盘 int16 * 3
                mag_x = struct.unpack('<h', frame[4:6])[0]
                mag_y = struct.unpack('<h', frame[6:8])[0]
                mag_z = struct.unpack('<h', frame[8:10])[0]
                self.sensor_data['mag'] = [mag_x, mag_y, mag_z]
                # 解析气压高度 int32 (cm)
                alt_bar = struct.unpack('<i', frame[10:14])[0]
                self.sensor_data['alt_bar'] = alt_bar
                # 解析温度 int16 (0.1°C)
                temp_raw = struct.unpack('<h', frame[14:16])[0]
                self.sensor_data['temp'] = temp_raw / 10.0  # 转换为°C
                # 状态字节
                self.sensor_data['bar_sta'] = frame[16]
                self.sensor_data['mag_sta'] = frame[17]

            # 记录数据
            self.append_data_log()

        except Exception as e:
            # 解析异常忽略，避免程序崩溃
            pass

    def receive_data_thread(self):
        """ANO协议串口数据接收线程，帧头0xAA + 地址0xFF"""
        frame_header = b'\xAA\xFF'
        buffer = b''

        while self.is_thread_running:
            try:
                if self.serial_port and self.serial_port.is_open and self.serial_port.in_waiting > 0:
                    # 读取串口缓存数据
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    buffer += data

                    # 循环查找完整ANO帧
                    while frame_header in buffer:
                        header_idx = buffer.index(frame_header)
                        # 至少读取到帧头+地址+功能码+长度+校验位，才判断长度
                        if len(buffer) - header_idx < 5:
                            break
                        # 获取数据长度，计算整帧长度
                        data_len = buffer[header_idx + 3]
                        full_frame_len = data_len + 6  # 帧头(2)+地址(1)+功能码(1)+数据+校验(2)
                        # 数据足够完整帧
                        if len(buffer) - header_idx >= full_frame_len:
                            full_frame = buffer[header_idx:header_idx + full_frame_len]
                            # 移除已处理数据
                            buffer = buffer[header_idx + full_frame_len:]
                            # 校验通过再解析
                            if self.ano_verify_checksum(full_frame):
                                self.parse_ano_frame(full_frame)
                        else:
                            break
                time.sleep(0.005)
            except Exception as e:
                if self.is_thread_running:
                    self.log_message(f"数据接收异常：{str(e)}")
                    time.sleep(0.5)
    # ======================================================================

    def simulate_data_thread(self):
        """模拟ANO协议数据生成线程，无硬件时测试"""
        while self.is_thread_running and self.simulate_mode:
            try:
                # 生成平滑的随机姿态角，适配修正后的旋转方向
                self.euler_angles['roll'] = np.clip(self.euler_angles['roll'] + random.uniform(-1.5, 1.5), -45, 45)
                self.euler_angles['pitch'] = np.clip(self.euler_angles['pitch'] + random.uniform(-1.2, 1.2), -45, 45)
                self.euler_angles['yaw'] += random.uniform(-0.8, 0.8)
                # 偏航角归一化
                if self.euler_angles['yaw'] > 180:
                    self.euler_angles['yaw'] -= 360
                elif self.euler_angles['yaw'] < -180:
                    self.euler_angles['yaw'] += 360

                # 模拟IMU传感器数据
                self.imu_raw['acc'] = [
                    random.uniform(-0.2, 0.2),
                    random.uniform(-0.2, 0.2),
                    random.uniform(0.8, 1.2)
                ]
                self.imu_raw['gyro'] = [
                    random.uniform(-5, 5),
                    random.uniform(-5, 5),
                    random.uniform(-3, 3)
                ]

                # 模拟遥控通道
                self.update_rc_channels(simulate=True)
                # 记录数据
                self.append_data_log()
                time.sleep(0.05)
            except Exception:
                time.sleep(0.1)

    def update_rc_channels(self, simulate=False):
        """更新遥控通道数据，模拟模式随机波动"""
        if simulate:
            for i in range(9):
                # 遥控器通道正常范围1000-2000，中位1500
                self.rc_channels[i] = int(np.clip(self.rc_channels[i] + random.randint(-10, 10), 1000, 2000))

    def append_data_log(self):
        """追加数据到记录列表"""
        if not self.is_recording:
            return
        current_time = time.time() - self.record_start_time
        log_data = {
            'time': round(current_time, 2),
            'roll': round(self.euler_angles['roll'], 2),
            'pitch': round(self.euler_angles['pitch'], 2),
            'yaw': round(self.euler_angles['yaw'], 2),
            'acc_x': round(self.imu_raw['acc'][0], 3),
            'acc_y': round(self.imu_raw['acc'][1], 3),
            'acc_z': round(self.imu_raw['acc'][2], 3),
            'gyro_x': round(self.imu_raw['gyro'][0], 2),
            'gyro_y': round(self.imu_raw['gyro'][1], 2),
            'gyro_z': round(self.imu_raw['gyro'][2], 2),
            'shock': self.shock_status,
            'rc_channels': self.rc_channels.copy()
        }
        self.data_log.append(log_data)

    def toggle_recording(self):
        """切换数据记录状态，保存为CSV文件"""
        if not self.is_connected:
            messagebox.showwarning("提示", "请先连接串口或开启模拟模式")
            return

        self.is_recording = not self.is_recording
        if self.is_recording:
            self.record_btn.config(text="停止记录")
            self.record_start_time = time.time()
            self.data_log.clear()
            self.log_message("开始记录ANO协议飞行数据...")
        else:
            self.record_btn.config(text="开始记录")
            self.log_message(f"停止记录，共采集{len(self.data_log)}条数据")
            # 保存为CSV文件
            if self.data_log:
                self.save_data_to_csv()

    def save_data_to_csv(self):
        """将记录的数据保存为CSV文件"""
        try:
            filename = f"ANO飞控数据_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            with open(filename, 'w', newline='', encoding='utf-8-sig') as f:
                # CSV表头
                headers = ['时间(s)', 'Roll(°)', 'Pitch(°)', 'Yaw(°)',
                           'AccX(g)', 'AccY(g)', 'AccZ(g)',
                           'GyroX(°/s)', 'GyroY(°/s)', 'GyroZ(°/s)', '震动状态']
                headers += [f'CH{i}' for i in range(9)]
                writer = csv.DictWriter(f, fieldnames=headers)
                writer.writeheader()

                # 写入数据
                for data in self.data_log:
                    row = {
                        '时间(s)': data['time'],
                        'Roll(°)': data['roll'],
                        'Pitch(°)': data['pitch'],
                        'Yaw(°)': data['yaw'],
                        'AccX(g)': data['acc_x'],
                        'AccY(g)': data['acc_y'],
                        'AccZ(g)': data['acc_z'],
                        'GyroX(°/s)': data['gyro_x'],
                        'GyroY(°/s)': data['gyro_y'],
                        'GyroZ(°/s)': data['gyro_z'],
                        '震动状态': data['shock']
                    }
                    for i in range(9):
                        row[f'CH{i}'] = data['rc_channels'][i]
                    writer.writerow(row)
            self.log_message(f"数据已保存至：{filename}")
        except Exception as e:
            self.log_message(f"保存数据失败：{str(e)}")

    def update_3d_attitude(self):
        """优化升级3D无人机姿态可视化，替换简陋模型为精致穿越机模型
        新增：中心机身、加粗机臂、前后区分、螺旋桨模块、机头方向箭头
        优化：配色区分、线条粗细、姿态旋转逻辑，修正俯仰横滚方向，贴合真实穿越机操控直觉
        核心修正：反转Pitch、Roll旋转矩阵符号，抬俯仰显示抬头，横滚方向匹配操控
        """
        self.init_3d_plot()
        # 角度转弧度，核心修正：反转俯仰Pitch、横滚Roll角度，匹配操控直觉
        # 原角度为负转正、正转负，解决抬头变低头、横滚反向问题
        roll = np.radians(-self.euler_angles['roll'])
        pitch = np.radians(-self.euler_angles['pitch'])
        yaw = np.radians(self.euler_angles['yaw'])

        # 优化后的穿越机尺寸参数，比例更协调
        arm_length = 1.6       # 机臂长度
        arm_thick = 2.5        # 机臂线条粗细
        body_radius = 0.3      # 中心机身半径
        prop_radius = 0.5      # 螺旋桨半径
        body_height = 1.0      # 中心机身高度

        # 旋转矩阵计算，修正旋转方向后保持原有姿态旋转顺序不变
        R_roll = np.array([[1, 0, 0],
                           [0, np.cos(roll), -np.sin(roll)],
                           [0, np.sin(roll), np.cos(roll)]])
        R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                            [0, 1, 0],
                            [-np.sin(pitch), 0, np.cos(pitch)]])
        R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                          [np.sin(yaw), np.cos(yaw), 0],
                          [0, 0, 1]])
        # 总旋转矩阵：偏航→俯仰→横滚，符合无人机标准姿态旋转顺序
        R = R_yaw @ R_pitch @ R_roll

        # 1. 绘制中心机身主体（立方体，模拟飞控核心板）
        body_points = np.array([
            [-body_radius, -body_radius, 0],
            [body_radius, -body_radius, 0],
            [body_radius, body_radius, 0],
            [-body_radius, body_radius, 0],
            [-body_radius, -body_radius, body_height],
            [body_radius, -body_radius, body_height],
            [body_radius, body_radius, body_height],
            [-body_radius, body_radius, body_height]
        ])
        rotated_body = (R @ body_points.T).T
        # 绘制机身边框
        body_edges = [
            [0,1], [1,2], [2,3], [3,0],
            [4,5], [5,6], [6,7], [7,4],
            [0,4], [1,5], [2,6], [3,7]
        ]
        for edge in body_edges:
            self.ax_3d.plot(
                [rotated_body[edge[0], 0], rotated_body[edge[1], 0]],
                [rotated_body[edge[0], 1], rotated_body[edge[1], 1]],
                [rotated_body[edge[0], 2], rotated_body[edge[1], 2]],
                color='#2C3E50', linewidth=3
            )

        # 2. 绘制四根机臂，区分前后机臂颜色，加粗线条
        # 机臂端点定义：前左、前右、后左、后右
        arm_endpoints = np.array([
            [arm_length, 0, body_height/2],   # 前右
            [-arm_length, 0, body_height/2],  # 后左
            [0, arm_length, body_height/2],   # 右前
            [0, -arm_length, body_height/2]   # 左后
        ])
        rotated_arms = (R @ arm_endpoints.T).T
        body_center = np.array([0, 0, body_height/2])
        rotated_center = (R @ body_center.T).T

        # 机臂配色：前机臂红色，后机臂蓝色，区分方向
        arm_colors = ['#E74C3C', '#3498DB', '#1ABC9C', '#9B59B6']
        for i in range(4):
            self.ax_3d.plot(
                [rotated_center[0], rotated_arms[i, 0]],
                [rotated_center[1], rotated_arms[i, 1]],
                [rotated_center[2], rotated_arms[i, 2]],
                color=arm_colors[i], linewidth=arm_thick
            )

        # 3. 绘制螺旋桨模块，每根机臂末端一个螺旋桨
        prop_colors = ['#FF5733', '#33A1FF', '#33FF57', '#FF33A6']
        for i in range(4):
            prop_center = rotated_arms[i]
            # 螺旋桨十字叶片
            prop_blades = np.array([
                [prop_radius, 0, 0], [-prop_radius, 0, 0],
                [0, prop_radius, 0], [0, -prop_radius, 0]
            ])
            rotated_blades = (R @ prop_blades.T).T + prop_center
            self.ax_3d.plot(
                [rotated_blades[0,0], rotated_blades[1,0]],
                [rotated_blades[0,1], rotated_blades[1,1]],
                [rotated_blades[0,2], rotated_blades[1,2]],
                color=prop_colors[i], linewidth=2
            )
            self.ax_3d.plot(
                [rotated_blades[2,0], rotated_blades[3,0]],
                [rotated_blades[2,1], rotated_blades[3,1]],
                [rotated_blades[2,2], rotated_blades[3,2]],
                color=prop_colors[i], linewidth=2
            )

        # 4. 绘制机头方向箭头，明确无人机前方，避免姿态混淆
        arrow_start = rotated_center
        arrow_end = (R @ np.array([arm_length*0.8, 0, body_height/2]).T).T
        self.ax_3d.plot(
            [arrow_start[0], arrow_end[0]],
            [arrow_start[1], arrow_end[1]],
            [arrow_start[2], arrow_end[2]],
            color='#F1C40F', linewidth=4, linestyle='-'
        )
        # 箭头尖端
        self.ax_3d.scatter(arrow_end[0], arrow_end[1], arrow_end[2], color='#F1C40F', s=50)

        # 强制刷新画布，提升渲染流畅度
        self.canvas_3d.draw()

    def update_2d_curve(self):
        """更新2D四元数实时曲线"""
        current_time = time.time() - self.start_plot_time
        # 追加最新四元数数据
        self.time_history.append(current_time)
        q0, q1, q2, q3 = self.quaternion
        self.roll_history.append(q0)
        self.pitch_history.append(q1)
        self.yaw_history.append(q2)

        # 需要更多历史数据存储q3
        if not hasattr(self, 'q3_history'):
            self.q3_history = []
        self.q3_history.append(q3)

        # 限制历史数据长度，防止内存溢出
        if len(self.time_history) > self.max_history:
            self.time_history.pop(0)
            self.roll_history.pop(0)
            self.pitch_history.pop(0)
            self.yaw_history.pop(0)
            if hasattr(self, 'q3_history'):
                self.q3_history.pop(0)

        # 更新曲线数据
        self.line_q0.set_data(self.time_history, self.roll_history)
        self.line_q1.set_data(self.time_history, self.pitch_history)
        self.line_q2.set_data(self.time_history, self.yaw_history)
        if hasattr(self, 'q3_history'):
            self.line_q3.set_data(self.time_history, self.q3_history)
        # 动态调整X轴范围
        self.ax_2d.set_xlim(max(0, current_time - self.max_history * 0.05), current_time)
        self.canvas_2d.draw()

    def update_imu_curve(self):
        """更新IMU原始数据波形曲线"""
        current_time = time.time() - self.start_plot_time
        # 追加最新数据
        acc_data = self.imu_raw['acc']
        gyro_data = self.imu_raw['gyro']
        self.acc_x_history.append(acc_data[0])
        self.acc_y_history.append(acc_data[1])
        self.acc_z_history.append(acc_data[2])
        self.gyro_x_history.append(gyro_data[0])
        self.gyro_y_history.append(gyro_data[1])
        self.gyro_z_history.append(gyro_data[2])

        # 限制历史数据长度
        if len(self.acc_x_history) > self.max_history:
            self.acc_x_history.pop(0)
            self.acc_y_history.pop(0)
            self.acc_z_history.pop(0)
            self.gyro_x_history.pop(0)
            self.gyro_y_history.pop(0)
            self.gyro_z_history.pop(0)

        # 更新曲线数据
        self.line_acc_x.set_data(self.time_history, self.acc_x_history)
        self.line_acc_y.set_data(self.time_history, self.acc_y_history)
        self.line_acc_z.set_data(self.time_history, self.acc_z_history)
        self.line_gyro_x.set_data(self.time_history, self.gyro_x_history)
        self.line_gyro_y.set_data(self.time_history, self.gyro_y_history)
        self.line_gyro_z.set_data(self.time_history, self.gyro_z_history)

        # 动态调整X轴范围
        self.ax_imu.set_xlim(max(0, current_time - self.max_history * 0.05), current_time)
        # 自动调整Y轴范围
        if self.acc_x_history:
            all_acc = self.acc_x_history + self.acc_y_history + self.acc_z_history
            all_gyro = self.gyro_x_history + self.gyro_y_history + self.gyro_z_history
            y_min = min(min(all_acc), min(all_gyro)) * 1.2
            y_max = max(max(all_acc), max(all_gyro)) * 1.2
            self.ax_imu.set_ylim(y_min, y_max)
        self.canvas_imu.draw()

    def update_display(self):
        """定时刷新GUI界面数据"""
        try:
            if self.is_connected:
                # 更新姿态角数值
                for angle in ['roll', 'pitch', 'yaw']:
                    self.attitude_labels[angle].config(text=f"{self.euler_angles[angle]:.2f}")

                # 更新遥控通道
                for i in range(9):
                    self.rc_labels[i].config(text=str(self.rc_channels[i]))

                # 更新传感器数据
                acc_data = self.imu_raw['acc']
                gyro_data = self.imu_raw['gyro']
                for i in range(3):
                    self.acc_labels[i].config(text=f"{acc_data[i]:.3f}")
                    self.gyro_labels[i].config(text=f"{gyro_data[i]:.2f}")

                # 更新气压计和温度数据
                self.alt_label.config(text=f"{self.sensor_data['alt_bar']} cm")
                self.temp_label.config(text=f"{self.sensor_data['temp']:.1f} °C")

                # 更新可视化图形
                self.update_3d_attitude()
                self.update_2d_curve()

        except Exception as e:
            pass

        # 定时刷新，50ms刷新一次
        self.root.after(50, self.update_display)

    def on_close(self):
        """关闭窗口时的清理操作"""
        try:
            self.is_thread_running = False
            self.is_recording = False
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            self.log_message("地面站已关闭")
            self.root.destroy()
        except:
            self.root.destroy()

    def run(self):
        """启动地面站长程序"""
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()


if __name__ == "__main__":
    # 启动程序
    ground_station = DroneGroundStation()
    ground_station.run()