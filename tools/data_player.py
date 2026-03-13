#!/usr/bin/env python3
"""
飞行数据回放工具
用于回放和分析之前记录的飞行数据
"""

import tkinter as tk
from tkinter import ttk, filedialog, scrolledtext
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from datetime import datetime
import os

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False

class DataPlayer:
    def __init__(self):
        self.data = []
        self.current_index = 0
        self.is_playing = False
        self.play_speed = 1.0

        self.setup_gui()

    def setup_gui(self):
        """设置GUI界面"""
        self.root = tk.Tk()
        self.root.title("飞行数据回放工具")
        self.root.geometry("1200x800")

        # 设置中文字体
        default_font = ('Microsoft YaHei', 9)
        self.root.option_add('*Font', default_font)

        # 主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # 控制区域
        control_frame = ttk.LabelFrame(main_frame, text="控制面板")
        control_frame.pack(fill=tk.X, pady=(0, 10))

        # 文件选择
        ttk.Button(control_frame, text="打开数据文件", command=self.load_data_file).grid(row=0, column=0, padx=5, pady=5)
        self.file_label = ttk.Label(control_frame, text="未选择文件")
        self.file_label.grid(row=0, column=1, padx=5, pady=5, sticky='w')

        # 播放控制
        ttk.Button(control_frame, text="播放/暂停", command=self.toggle_playback).grid(row=0, column=2, padx=5, pady=5)
        ttk.Button(control_frame, text="停止", command=self.stop_playback).grid(row=0, column=3, padx=5, pady=5)

        # 速度控制
        ttk.Label(control_frame, text="播放速度:").grid(row=0, column=4, padx=5, pady=5)
        self.speed_var = tk.StringVar(value="1.0")
        speed_combo = ttk.Combobox(control_frame, textvariable=self.speed_var,
                                  values=["0.25", "0.5", "1.0", "2.0", "4.0"], width=5)
        speed_combo.grid(row=0, column=5, padx=5, pady=5)
        speed_combo.bind('<<ComboboxSelected>>', self.update_play_speed)

        # 进度条
        self.progress_var = tk.DoubleVar()
        self.progress_scale = ttk.Scale(control_frame, from_=0, to=100, variable=self.progress_var,
                                       orient=tk.HORIZONTAL, command=self.seek_data)
        self.progress_scale.grid(row=1, column=0, columnspan=6, sticky='ew', padx=5, pady=5)

        # 数据显示区域
        display_frame = ttk.Frame(main_frame)
        display_frame.pack(fill=tk.BOTH, expand=True)

        # 左侧：数据显示
        left_frame = ttk.Frame(display_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        # 当前数据显示
        data_frame = ttk.LabelFrame(left_frame, text="当前数据")
        data_frame.pack(fill=tk.X, pady=(0, 10))

        # 姿态角
        ttk.Label(data_frame, text="姿态角:").grid(row=0, column=0, sticky='w', padx=5, pady=2)
        self.roll_label = ttk.Label(data_frame, text="Roll: 0.00°", font=('Courier', 12))
        self.roll_label.grid(row=1, column=0, sticky='w', padx=10, pady=2)
        self.pitch_label = ttk.Label(data_frame, text="Pitch: 0.00°", font=('Courier', 12))
        self.pitch_label.grid(row=2, column=0, sticky='w', padx=10, pady=2)
        self.yaw_label = ttk.Label(data_frame, text="Yaw: 0.00°", font=('Courier', 12))
        self.yaw_label.grid(row=3, column=0, sticky='w', padx=10, pady=2)

        # 飞行状态
        ttk.Label(data_frame, text="数据状态:").grid(row=4, column=0, sticky='w', padx=5, pady=(10, 2))
        self.status_label = ttk.Label(data_frame, text="等待数据加载", font=('Courier', 10))
        self.status_label.grid(row=5, column=0, columnspan=3, sticky='w', padx=10, pady=1)

        # 传感器数据
        ttk.Label(data_frame, text="传感器数据:").grid(row=8, column=0, columnspan=3, sticky='w', padx=5, pady=(10, 2))
        self.acc_labels = []
        self.gyro_labels = []

        ttk.Label(data_frame, text="加速度计 (g):").grid(row=9, column=0, columnspan=3, sticky='w', padx=10, pady=1)
        for i, axis in enumerate(['X', 'Y', 'Z']):
            label = ttk.Label(data_frame, text=f"A{axis}: 0.000", font=('Courier', 9))
            label.grid(row=10, column=i, sticky='w', padx=5, pady=1)
            self.acc_labels.append(label)

        ttk.Label(data_frame, text="陀螺仪 (°/s):").grid(row=11, column=0, columnspan=3, sticky='w', padx=10, pady=1)
        for i, axis in enumerate(['X', 'Y', 'Z']):
            label = ttk.Label(data_frame, text=f"G{axis}: 0.000", font=('Courier', 9))
            label.grid(row=12, column=i, sticky='w', padx=5, pady=1)
            self.gyro_labels.append(label)

        # 右侧：图形显示
        right_frame = ttk.Frame(display_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))

        # 姿态角曲线
        fig = plt.figure(figsize=(10, 6))
        self.ax = fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(fig, right_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # 初始化曲线
        self.time_data = []
        self.roll_data = []
        self.pitch_data = []
        self.yaw_data = []
        self.current_line = None

        # 底部信息
        info_frame = ttk.LabelFrame(main_frame, text="文件信息")
        info_frame.pack(fill=tk.X, pady=(10, 0))

        self.info_text = scrolledtext.ScrolledText(info_frame, height=6, wrap=tk.WORD)
        self.info_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    def load_data_file(self):
        """加载数据文件"""
        filename = filedialog.askopenfilename(
            title="选择数据文件",
            filetypes=[("CSV文件", "*.csv"), ("所有文件", "*.*")]
        )

        if filename:
            try:
                self.data = []
                with open(filename, 'r', encoding='utf-8') as csvfile:
                    reader = csv.DictReader(csvfile)
                    for row in reader:
                        self.data.append(row)

                if self.data:
                    self.current_index = 0
                    self.progress_scale.config(to=len(self.data)-1)
                    self.progress_var.set(0)

                    self.file_label.config(text=os.path.basename(filename))
                    self.update_display()
                    self.plot_data()

                    # 显示文件信息
                    self.show_file_info(filename)

                    self.info_text.insert(tk.END, f"成功加载数据文件: {filename}\n")
                    self.info_text.insert(tk.END, f"数据点数: {len(self.data)}\n")

                else:
                    self.info_text.insert(tk.END, "文件为空或格式错误\n")

            except Exception as e:
                self.info_text.insert(tk.END, f"加载文件失败: {e}\n")

    def show_file_info(self, filename):
        """显示文件信息"""
        try:
            file_size = os.path.getsize(filename)
            file_time = datetime.fromtimestamp(os.path.getmtime(filename))

            self.info_text.insert(tk.END, f"文件大小: {file_size:,} 字节\n")
            self.info_text.insert(tk.END, f"创建时间: {file_time.strftime('%Y-%m-%d %H:%M:%S')}\n")

            if self.data:
                # 计算时间范围
                start_time = float(self.data[0]['timestamp'])
                end_time = float(self.data[-1]['timestamp'])
                duration = end_time - start_time

                self.info_text.insert(tk.END, f"记录时长: {duration:.1f} 秒\n")
                self.info_text.insert(tk.END, f"采样率: {len(self.data)/duration:.1f} Hz\n")

                # 显示数据统计
                self.show_data_statistics()

        except Exception as e:
            self.info_text.insert(tk.END, f"获取文件信息失败: {e}\n")

    def show_data_statistics(self):
        """显示数据统计信息"""
        try:
            # 提取数据
            roll_values = [float(row['roll']) for row in self.data]
            pitch_values = [float(row['pitch']) for row in self.data]
            yaw_values = [float(row['yaw']) for row in self.data]

            self.info_text.insert(tk.END, "\n姿态角统计:\n")
            self.info_text.insert(tk.END, f"Roll:  最小值={min(roll_values):6.1f}°  最大值={max(roll_values):6.1f}°  平均值={np.mean(roll_values):6.1f}°\n")
            self.info_text.insert(tk.END, f"Pitch: 最小值={min(pitch_values):6.1f}°  最大值={max(pitch_values):6.1f}°  平均值={np.mean(pitch_values):6.1f}°\n")
            self.info_text.insert(tk.END, f"Yaw:   最小值={min(yaw_values):6.1f}°  最大值={max(yaw_values):6.1f}°  平均值={np.mean(yaw_values):6.1f}°\n")

        except Exception as e:
            self.info_text.insert(tk.END, f"数据统计失败: {e}\n")

    def toggle_playback(self):
        """切换播放/暂停状态"""
        if not self.data:
            return

        self.is_playing = not self.is_playing

        if self.is_playing:
            self.play_data()

    def play_data(self):
        """播放数据"""
        if not self.is_playing or not self.data:
            return

        if self.current_index < len(self.data):
            self.update_display()
            self.current_index += 1
            self.progress_var.set(self.current_index)

            # 设置下一次播放
            delay = int(100 / self.play_speed)  # 基础10Hz
            self.root.after(delay, self.play_data)
        else:
            self.is_playing = False

    def stop_playback(self):
        """停止播放"""
        self.is_playing = False
        self.current_index = 0
        self.progress_var.set(0)
        self.update_display()

    def seek_data(self, value):
        """跳转到指定位置"""
        if self.data:
            self.current_index = int(float(value))
            self.update_display()

    def update_play_speed(self, event=None):
        """更新播放速度"""
        try:
            self.play_speed = float(self.speed_var.get())
        except:
            self.play_speed = 1.0

    def update_display(self):
        """更新数据显示"""
        if not self.data or self.current_index >= len(self.data):
            return

        row = self.data[self.current_index]

        # 更新姿态角
        self.roll_label.config(text=f"Roll:  {float(row['roll']):7.2f}°")
        self.pitch_label.config(text=f"Pitch: {float(row['pitch']):7.2f}°")
        self.yaw_label.config(text=f"Yaw:   {float(row['yaw']):7.2f}°")

        # 更新状态
        self.status_label.config(text=f"数据点: {self.current_index + 1} / {len(self.data)}")

        # 更新传感器数据
        for i, axis in enumerate(['x', 'y', 'z']):
            acc_key = f'acc_{axis}'
            gyro_key = f'gyro_{axis}'

            if acc_key in row:
                self.acc_labels[i].config(text=f"A{axis.upper()}: {float(row[acc_key]):7.3f}")
            if gyro_key in row:
                self.gyro_labels[i].config(text=f"G{axis.upper()}: {float(row[gyro_key]):7.3f}")

    def plot_data(self):
        """绘制数据曲线"""
        if not self.data:
            return

        self.ax.clear()

        # 准备数据
        time_values = []
        roll_values = []
        pitch_values = []
        yaw_values = []

        base_time = float(self.data[0]['timestamp'])

        for row in self.data:
            time_values.append(float(row['timestamp']) - base_time)
            roll_values.append(float(row['roll']))
            pitch_values.append(float(row['pitch']))
            yaw_values.append(float(row['yaw']))

        # 绘制曲线
        self.ax.plot(time_values, roll_values, 'r-', label='Roll', linewidth=2)
        self.ax.plot(time_values, pitch_values, 'g-', label='Pitch', linewidth=2)
        self.ax.plot(time_values, yaw_values, 'b-', label='Yaw', linewidth=2)

        # 绘制当前位置线
        if self.current_index < len(time_values):
            current_time = time_values[self.current_index]
            self.current_line = self.ax.axvline(x=current_time, color='black', linestyle='--', alpha=0.8)

        # 设置图形属性
        self.ax.set_xlabel('时间 (秒)')
        self.ax.set_ylabel('角度 (度)')
        self.ax.set_title('飞行姿态角曲线')
        self.ax.legend()
        self.ax.grid(True, alpha=0.3)
        self.ax.set_ylim([-180, 180])

        self.canvas.draw()

    def run(self):
        """运行数据播放器"""
        self.info_text.insert(tk.END, "飞行数据回放工具启动\n")
        self.info_text.insert(tk.END, "请打开一个数据文件开始分析\n")
        self.root.mainloop()

if __name__ == "__main__":
    try:
        app = DataPlayer()
        app.run()
    except Exception as e:
        print(f"启动失败: {e}")
        print("请确保已安装以下库:")
        print("pip install numpy matplotlib")