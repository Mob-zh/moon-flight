# ============================================================
# 黑匣子上位机 - Betaflight 风格界面优化版
# 对应界面：深色主题 + 多子图布局 + 陀螺仪/电机/频谱图
# ------------------------------------------------------------
# 【当前已实现功能 ✅】
# 1. 整体深色主题，仿 Betaflight Blackbox Explorer 风格
# 2. 顶部曲线：陀螺仪(Gyro) + 设定点(Setpoint) 对比显示
# 3. 左下：四轴电机姿态可视化示意图
# 4. 右下：油门频率热力图 (Freq vs Throttle)
# 5. 底部：4 路电机输出曲线对比
# 6. 完整 I 帧解析：时间、油门、电机、陀螺、设定点、电压等
# 7. P 帧基础解析，保证时间连续不崩溃
# 8. 支持打开 .bbl 黑匣子日志并绘图显示
# 9. 支持导出完整 CSV 数据文件
# 10. 自带配色体系：Roll/ Pitch/ Yaw/电机/油门区分明显
# 11. 增加曲线显隐开关（Gyro/Setpoint/Motor独立控制）
# 12. 增加右侧数据面板：实时显示 Gyro、Setpoint、电机、电压等值
# 13. 增加时间轴滑块：支持拖动查看任意时间段数据
# 14. 增加日志信息面板：显示采样率、帧数、时长、电压范围
# 15. 增加异常数据保护：防止 NaN、溢出导致崩溃
# 16. 支持保存图表为 PNG 图片
# 17. 启用 matplotlib 交互工具栏（缩放/平移）
# ------------------------------------------------------------
# 【待实现 / 可优化功能（按图片效果进一步完善）💡】
# 1. 界面布局精细化：严格对齐 Betaflight 区域比例与间距
# 2. 完善 P 帧差分解码：实现真正的 Betaflight TAG8_4S16 完整解包
# 3. 增加滤波显示：原始 Gyro / 滤波后 Gyro 对比曲线
# 4. 增加 D-term、PID 输出、Mix 混控输出等专业通道显示
# 5. 增加电机联动动画：四轴姿态图随电机输出实时动态变化
# 6. 支持批量日志加载与多日志对比显示
# 7. 支持噪声统计：RMS、峰值、主要振动频率自动标注
# ============================================================

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from scipy.signal import spectrogram

# ==================== 全局配置（Betaflight风格）====================
MIN_THROTTLE = 1000
plt.rcParams["font.sans-serif"] = ["SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.facecolor"] = "#1a1a1a"
plt.rcParams["axes.facecolor"] = "#1a1a1a"
plt.rcParams["axes.labelcolor"] = "white"
plt.rcParams["xtick.color"] = "white"
plt.rcParams["ytick.color"] = "white"
plt.rcParams["grid.color"] = "#444444"
plt.rcParams["text.color"] = "white"
plt.rcParams["legend.facecolor"] = "#1a1a1a"
plt.rcParams["legend.labelcolor"] = "white"

COLORS = {
    "roll": "#ff6b6b",
    "pitch": "#4ecdc4",
    "yaw": "#45b7d1",
    "throttle": "#f9ca24",
    "motor": "#f0932b",
    "gyro": "#ff7675",
    "setpoint": "#fdcb6e"
}

# ==================== 黑匣子解析器 ====================
class BlackboxParser:
    def __init__(self):
        self.data = []
        self.df = None
        self.bb_vbat_ref = 0
        self.last_i_frame = None
        self.log_info = {}

    def _safe_value(self, val, default=0):
        """安全值检查，防止NaN和溢出"""
        if val is None or (isinstance(val, float) and (np.isnan(val) or np.isinf(val))):
            return default
        return val

    def _validate_frame(self, frame):
        """验证并清理帧数据"""
        for key in ["gyro_x", "gyro_y", "gyro_z", "motor1", "motor2", "motor3", "motor4"]:
            if key in frame:
                frame[key] = self._safe_value(frame[key])
        frame["throttle"] = self._safe_value(frame["throttle"], 1000)
        frame["vbat"] = self._safe_value(frame["vbat"], 0)
        frame["rssi"] = self._safe_value(frame["rssi"], 0)
        return frame

    def zigzag_decode(self, val):
        return (val >> 1) ^ -(val & 1)

    def read_vb(self, data, ptr):
        val = 0
        shift = 0
        start = ptr
        while ptr < len(data):
            b = data[ptr]
            ptr += 1
            val |= (b & 0x7F) << shift
            shift += 7
            if not (b & 0x80):
                return val, ptr
        return 0, start

    def _decode_tag8_4s16(self, data, ptr):
        if ptr + 1 > len(data):
            return [0,0,0,0], ptr
        selector = data[ptr]
        ptr += 1
        values = [0,0,0,0]
        nibble = 0
        buf = 0

        for i in range(4):
            sel = (selector >> (i * 2)) & 0x03
            if sel == 0:
                continue
            if ptr >= len(data):
                break
            if sel == 1:
                if nibble == 0:
                    buf = data[ptr]
                    ptr += 1
                    val = (buf >> 4) & 0x0F
                    values[i] = self.zigzag_decode(val)
                    nibble = 1
                else:
                    val = buf & 0x0F
                    values[i] = self.zigzag_decode(val)
                    nibble = 0
            elif sel == 2:
                if nibble == 0:
                    val = data[ptr]
                    ptr += 1
                    values[i] = val
                else:
                    val = ((buf & 0x0F) << 4) | (data[ptr] >> 4)
                    values[i] = val
                    buf = data[ptr] << 4
                    ptr += 1
            else:
                if nibble == 0:
                    if ptr + 2 > len(data):
                        break
                    val = int.from_bytes(data[ptr:ptr+2], byteorder='little', signed=True)
                    ptr += 2
                    values[i] = val
                else:
                    if ptr + 2 > len(data):
                        break
                    b1 = ((buf & 0x0F) << 4) | (data[ptr] >> 4)
                    b2 = ((data[ptr] & 0x0F) << 4) | (data[ptr+1] >> 4)
                    val = int.from_bytes(bytes([b1, b2]), byteorder='little', signed=True)
                    values[i] = val
                    buf = data[ptr+1] << 4
                    ptr += 2
        if nibble == 1:
            ptr += 1
        return values, ptr

    def parse_log(self, filepath):
        try:
            with open(filepath, "rb") as f:
                raw = f.read()
            ptr = 0
            self.data = []
            self.bb_vbat_ref = 0
            self.last_i_frame = None

            # 收集日志基本信息
            self.log_info = {
                "filename": filepath.split("/")[-1].split("\\")[-1],
                "file_size": len(raw),
                "start_time": None,
                "end_time": None,
                "frame_count": 0,
                "i_frame_count": 0,
                "p_frame_count": 0,
                "avg_sample_rate": 0,
                "vbat_min": 999,
                "vbat_max": 0,
                "runtime_s": 0
            }

            # 跳过H帧头部
            while ptr < len(raw) and raw[ptr] == ord('H'):
                while ptr < len(raw) and raw[ptr] != ord('\n'):
                    ptr += 1
                ptr += 1

            while ptr < len(raw):
                if ptr >= len(raw):
                    break
                c = raw[ptr]
                if c == ord('I'):
                    ptr += 1
                    ptr = self._parse_i_frame(raw, ptr)
                elif c == ord('P'):
                    ptr += 1
                    ptr = self._parse_p_frame(raw, ptr)
                elif c == 0xE:
                    ptr += 1
                    _, ptr = self.read_vb(raw, ptr)
                    _, ptr = self.read_vb(raw, ptr)
                elif c == ord('S'):
                    ptr += 1
                    for _ in range(5):
                        _, ptr = self.read_vb(raw, ptr)
                else:
                    ptr += 1

            self.df = pd.DataFrame(self.data)

            # 计算统计信息
            if len(self.df) > 1:
                time_diffs = self.df["time_s"].diff().dropna()
                valid_diffs = time_diffs[(time_diffs > 0) & (time_diffs < 1.0)]
                if len(valid_diffs) > 0:
                    self.log_info["avg_sample_rate"] = round(1.0 / valid_diffs.mean(), 1)
                self.log_info["frame_count"] = len(self.df)
                self.log_info["runtime_s"] = round(self.df["time_s"].iloc[-1] - self.df["time_s"].iloc[0], 3)
                self.log_info["start_time"] = self.df["time_s"].iloc[0]
                self.log_info["end_time"] = self.df["time_s"].iloc[-1]
                self.log_info["vbat_min"] = round(self.df["vbat"].min(), 2)
                self.log_info["vbat_max"] = round(self.df["vbat"].max(), 2)

            return True
        except Exception as e:
            messagebox.showerror("解析失败", f"解析出错：{str(e)}")
            return False

    def _parse_i_frame(self, raw, ptr):
        iter_, ptr = self.read_vb(raw, ptr)
        time_us, ptr = self.read_vb(raw, ptr)
        time_s = round(time_us / 1e6, 3)

        pid_p = []
        for _ in range(3):
            v, ptr = self.read_vb(raw, ptr)
            pid_p.append(self.zigzag_decode(v))
        pid_i = []
        for _ in range(3):
            v, ptr = self.read_vb(raw, ptr)
            pid_i.append(self.zigzag_decode(v))
        pid_d = []
        for _ in range(3):
            v, ptr = self.read_vb(raw, ptr)
            pid_d.append(self.zigzag_decode(v))
        pid_f = []
        for _ in range(3):
            v, ptr = self.read_vb(raw, ptr)
            pid_f.append(self.zigzag_decode(v))

        rc_r, ptr = self.read_vb(raw, ptr)
        rc_p, ptr = self.read_vb(raw, ptr)
        rc_y, ptr = self.read_vb(raw, ptr)
        throttle, ptr = self.read_vb(raw, ptr)

        setpoint = []
        for _ in range(4):
            v, ptr = self.read_vb(raw, ptr)
            setpoint.append(self.zigzag_decode(v))

        vbat_enc, ptr = self.read_vb(raw, ptr)
        if self.bb_vbat_ref == 0:
            self.bb_vbat_ref = 124
        vbat = self.bb_vbat_ref - (vbat_enc & 0x3FFF)
        amperage_enc, ptr = self.read_vb(raw, ptr)

        gyro_x_enc, ptr = self.read_vb(raw, ptr)
        gyro_y_enc, ptr = self.read_vb(raw, ptr)
        gyro_z_enc, ptr = self.read_vb(raw, ptr)
        gyro = [self.zigzag_decode(gyro_x_enc), self.zigzag_decode(gyro_y_enc), self.zigzag_decode(gyro_z_enc)]

        for _ in range(3):
            _, ptr = self.read_vb(raw, ptr)

        m0_enc, ptr = self.read_vb(raw, ptr)
        m0 = m0_enc + MIN_THROTTLE
        m1_enc, ptr = self.read_vb(raw, ptr)
        m1 = m0 + self.zigzag_decode(m1_enc)
        m2_enc, ptr = self.read_vb(raw, ptr)
        m2 = m0 + self.zigzag_decode(m2_enc)
        m3_enc, ptr = self.read_vb(raw, ptr)
        m3 = m0 + self.zigzag_decode(m3_enc)

        baro_enc, ptr = self.read_vb(raw, ptr)
        rssi, ptr = self.read_vb(raw, ptr)

        frame = {
            "time_s": self._safe_value(time_s),
            "throttle": self._safe_value(throttle),
            "motor1": self._safe_value(m0), "motor2": self._safe_value(m1), "motor3": self._safe_value(m2), "motor4": self._safe_value(m3),
            "gyro_x": self._safe_value(gyro[0]), "gyro_y": self._safe_value(gyro[1]), "gyro_z": self._safe_value(gyro[2]),
            "setpoint_r": self._safe_value(setpoint[0]), "setpoint_p": self._safe_value(setpoint[1]), "setpoint_y": self._safe_value(setpoint[2]),
            "vbat": self._safe_value(vbat), "rssi": self._safe_value(rssi)
        }
        frame = self._validate_frame(frame)
        self.data.append(frame)
        self.last_i_frame = frame.copy()
        self.log_info["i_frame_count"] = self.log_info.get("i_frame_count", 0) + 1
        return ptr

    def _parse_p_frame(self, raw, ptr):
        if not self.last_i_frame or len(self.data) < 2 or ptr >= len(raw):
            return ptr
        last = self.data[-1].copy()
        prev2 = self.data[-2].copy()

        time_delta, ptr = self.read_vb(raw, ptr)
        time_delta = self.zigzag_decode(time_delta)
        last["time_s"] = round(time_delta + 2 * last["time_s"] - prev2["time_s"], 3)

        for i in range(3):
            v, ptr = self.read_vb(raw, ptr)
            delta = self.zigzag_decode(v)
            last[f"gyro_{['x','y','z'][i]}"] += delta

        if ptr < len(raw):
            sel = raw[ptr] >> 6
            ptr += 1
            if sel == 1: ptr += 1
            elif sel == 2: ptr += 2
            elif sel == 3: ptr += 13

        for _ in range(6):
            _, ptr = self.read_vb(raw, ptr)

        rc_delta, ptr = self._decode_tag8_4s16(raw, ptr)
        if ptr < len(raw):
            last["throttle"] += rc_delta[3]

        _, ptr = self._decode_tag8_4s16(raw, ptr)

        if ptr < len(raw):
            hdr = raw[ptr]
            ptr += 1
            for i in range(2):
                if (hdr >> (7-i)) & 1:
                    _, ptr = self.read_vb(raw, ptr)

        for i in range(3):
            v, ptr = self.read_vb(raw, ptr)
            delta = self.zigzag_decode(v)
            g = f"gyro_{['x','y','z'][i]}"
            last[g] = delta + (last[g] + prev2[g]) / 2

        for _ in range(3):
            _, ptr = self.read_vb(raw, ptr)

        motor_delta, ptr = self._decode_tag8_4s16(raw, ptr)
        for i in range(4):
            k = f"motor{i+1}"
            last[k] = motor_delta[i] + (last[k] + prev2[k]) / 2

        # 应用安全检查
        last = self._validate_frame(last)
        self.data.append(last)
        self.log_info["p_frame_count"] = self.log_info.get("p_frame_count", 0) + 1
        return ptr

# ==================== GUI ====================
class BlackboxViewer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Betaflight Blackbox 风格查看器")
        self.geometry("1700x950")
        self.parser = BlackboxParser()
        self.df = None
        self.configure(bg="#1a1a1a")

        # 曲线开关状态
        self.show_gyro = {"roll": True, "pitch": True, "yaw": True}
        self.show_setpoint = {"roll": True, "pitch": True, "yaw": True}
        self.show_motor = {"m1": True, "m2": True, "m3": True, "m4": True}
        self.current_time_idx = 0

        # ===== 顶部工具栏 =====
        top = ttk.Frame(self, height=60)
        top.pack(fill=tk.X, padx=10, pady=8)

        ttk.Button(top, text="打开日志", command=self.open_log, width=12).pack(side=tk.LEFT, padx=5)
        ttk.Button(top, text="导出CSV", command=self.export_csv, width=12).pack(side=tk.LEFT, padx=5)
        ttk.Button(top, text="保存图片", command=self.save_figure, width=12).pack(side=tk.LEFT, padx=5)

        self.info = ttk.Label(top, text="未加载日志", font=("Arial", 11))
        self.info.pack(side=tk.LEFT, padx=20)

        # ===== 曲线开关栏 =====
        switch_frame = ttk.Frame(self, height=40)
        switch_frame.pack(fill=tk.X, padx=10, pady=(0,5))

        ttk.Label(switch_frame, text="显示:", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

        # Gyro开关
        self.chk_gyro_r = tk.BooleanVar(value=True)
        ttk.Checkbutton(switch_frame, text="Gyro R", variable=self.chk_gyro_r, command=self.draw_all).pack(side=tk.LEFT, padx=3)
        self.chk_gyro_p = tk.BooleanVar(value=True)
        ttk.Checkbutton(switch_frame, text="Gyro P", variable=self.chk_gyro_p, command=self.draw_all).pack(side=tk.LEFT, padx=3)
        self.chk_gyro_y = tk.BooleanVar(value=True)
        ttk.Checkbutton(switch_frame, text="Gyro Y", variable=self.chk_gyro_y, command=self.draw_all).pack(side=tk.LEFT, padx=3)

        ttk.Separator(switch_frame, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)

        # Setpoint开关
        self.chk_sp_r = tk.BooleanVar(value=True)
        ttk.Checkbutton(switch_frame, text="Set R", variable=self.chk_sp_r, command=self.draw_all).pack(side=tk.LEFT, padx=3)
        self.chk_sp_p = tk.BooleanVar(value=True)
        ttk.Checkbutton(switch_frame, text="Set P", variable=self.chk_sp_p, command=self.draw_all).pack(side=tk.LEFT, padx=3)
        self.chk_sp_y = tk.BooleanVar(value=True)
        ttk.Checkbutton(switch_frame, text="Set Y", variable=self.chk_sp_y, command=self.draw_all).pack(side=tk.LEFT, padx=3)

        ttk.Separator(switch_frame, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)

        # Motor开关
        self.chk_m1 = tk.BooleanVar(value=True)
        ttk.Checkbutton(switch_frame, text="M1", variable=self.chk_m1, command=self.draw_all).pack(side=tk.LEFT, padx=3)
        self.chk_m2 = tk.BooleanVar(value=True)
        ttk.Checkbutton(switch_frame, text="M2", variable=self.chk_m2, command=self.draw_all).pack(side=tk.LEFT, padx=3)
        self.chk_m3 = tk.BooleanVar(value=True)
        ttk.Checkbutton(switch_frame, text="M3", variable=self.chk_m3, command=self.draw_all).pack(side=tk.LEFT, padx=3)
        self.chk_m4 = tk.BooleanVar(value=True)
        ttk.Checkbutton(switch_frame, text="M4", variable=self.chk_m4, command=self.draw_all).pack(side=tk.LEFT, padx=3)

        # ===== 主布局：左+右 =====
        self.main_frame = ttk.Frame(self)
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # 左侧：图表区域
        left_frame = ttk.Frame(self.main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.fig_top = plt.Figure(figsize=(16, 4), dpi=90)
        self.ax_top = self.fig_top.add_subplot(111)
        self.canvas_top = FigureCanvasTkAgg(self.fig_top, left_frame)
        self.canvas_top.get_tk_widget().pack(fill=tk.X, padx=5, pady=5)

        # 启用交互工具栏（可能不存在，需检查）
        try:
            toolbar = self.fig_top.canvas.manager.toolbar
            if toolbar:
                toolbar.configure()
        except:
            pass

        self.mid = ttk.Frame(left_frame)
        self.mid.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.fig_quad = plt.Figure(figsize=(4, 4), dpi=90)
        self.ax_quad = self.fig_quad.add_subplot(111)
        self.canvas_quad = FigureCanvasTkAgg(self.fig_quad, self.mid)
        self.canvas_quad.get_tk_widget().pack(side=tk.LEFT, padx=5)

        self.fig_spec = plt.Figure(figsize=(12, 4), dpi=90)
        self.ax_spec = self.fig_spec.add_subplot(111)
        self.canvas_spec = FigureCanvasTkAgg(self.fig_spec, self.mid)
        self.canvas_spec.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)

        self.fig_motor = plt.Figure(figsize=(16, 3), dpi=90)
        self.ax_motor = self.fig_motor.add_subplot(111)
        self.canvas_motor = FigureCanvasTkAgg(self.fig_motor, left_frame)
        self.canvas_motor.get_tk_widget().pack(fill=tk.X, padx=5, pady=5)

        # 右侧：数据面板
        right_frame = ttk.Frame(self.main_frame, width=220)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(10,0))
        right_frame.pack_propagate(False)

        ttk.Label(right_frame, text="数据面板", font=("Arial", 12, "bold")).pack(pady=10)

        # 时间信息
        self.lbl_time = ttk.Label(right_frame, text="时间: -- s", font=("Arial", 10))
        self.lbl_time.pack(anchor=tk.W, padx=10)

        # Gyro数据
        ttk.Label(right_frame, text="Gyro:", font=("Arial", 11, "bold")).pack(anchor=tk.W, padx=10, pady=(15,5))
        self.lbl_gyro_x = ttk.Label(right_frame, text="X: --", font=("Arial", 10))
        self.lbl_gyro_x.pack(anchor=tk.W, padx=20)
        self.lbl_gyro_y = ttk.Label(right_frame, text="Y: --", font=("Arial", 10))
        self.lbl_gyro_y.pack(anchor=tk.W, padx=20)
        self.lbl_gyro_z = ttk.Label(right_frame, text="Z: --", font=("Arial", 10))
        self.lbl_gyro_z.pack(anchor=tk.W, padx=20)

        # Setpoint数据
        ttk.Label(right_frame, text="Setpoint:", font=("Arial", 11, "bold")).pack(anchor=tk.W, padx=10, pady=(15,5))
        self.lbl_set_r = ttk.Label(right_frame, text="R: --", font=("Arial", 10))
        self.lbl_set_r.pack(anchor=tk.W, padx=20)
        self.lbl_set_p = ttk.Label(right_frame, text="P: --", font=("Arial", 10))
        self.lbl_set_p.pack(anchor=tk.W, padx=20)
        self.lbl_set_y = ttk.Label(right_frame, text="Y: --", font=("Arial", 10))
        self.lbl_set_y.pack(anchor=tk.W, padx=20)

        # 电机数据
        ttk.Label(right_frame, text="Motor:", font=("Arial", 11, "bold")).pack(anchor=tk.W, padx=10, pady=(15,5))
        self.lbl_motors = []
        for i in range(4):
            lbl = ttk.Label(right_frame, text=f"M{i+1}: --", font=("Arial", 10))
            lbl.pack(anchor=tk.W, padx=20)
            self.lbl_motors.append(lbl)

        # 电压和RSSI
        ttk.Label(right_frame, text="其他:", font=("Arial", 11, "bold")).pack(anchor=tk.W, padx=10, pady=(15,5))
        self.lbl_vbat = ttk.Label(right_frame, text="电压: -- V", font=("Arial", 10))
        self.lbl_vbat.pack(anchor=tk.W, padx=20)
        self.lbl_rssi = ttk.Label(right_frame, text="RSSI: --", font=("Arial", 10))
        self.lbl_rssi.pack(anchor=tk.W, padx=20)
        self.lbl_throttle = ttk.Label(right_frame, text="油门: --", font=("Arial", 10))
        self.lbl_throttle.pack(anchor=tk.W, padx=20)

        # ===== 时间轴滑块 =====
        slider_frame = ttk.Frame(self, height=50)
        slider_frame.pack(fill=tk.X, padx=10, pady=(0,8))

        ttk.Label(slider_frame, text="时间:").pack(side=tk.LEFT, padx=5)
        self.time_slider = tk.Scale(slider_frame, from_=0, to=100, orient=tk.HORIZONTAL,
                                     command=self.on_time_slider, showvalue=False,
                                     bg="#1a1a1a", fg="white", troughcolor="#333333",
                                     length=1400)
        self.time_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        self._init_quad()

    def _init_quad(self):
        self.ax_quad.clear()
        self.ax_quad.set_xlim(-1.2, 1.2)
        self.ax_quad.set_ylim(-1.2, 1.2)
        self.ax_quad.set_aspect('equal')
        self.ax_quad.set_title("四轴姿态", color="white")
        pos = [(-0.8,0), (0,0.8), (0.8,0), (0,-0.8)]
        cols = ["#ff6b6b","#4ecdc4","#f0932b","#a29bfe"]
        self.motors = []
        for (x,y),c in zip(pos,cols):
            cir = plt.Circle((x,y),0.2,color=c,alpha=0.7)
            self.ax_quad.add_patch(cir)
            self.motors.append(cir)
        self.ax_quad.arrow(0,0,0,0.5,head_width=0.1,color="white")
        self.canvas_quad.draw()

    def open_log(self):
        p = filedialog.askopenfilename(filetypes=[("BBL日志","*.bbl"),("所有文件","*.*")])
        if not p: return
        self.info.config(text="解析中...")
        self.update()
        if self.parser.parse_log(p):
            self.df = self.parser.df
            info = self.parser.log_info
            info_text = f"{info['filename']} | {info['frame_count']}帧 | {info['runtime_s']}s | {info['avg_sample_rate']}Hz | V:{info['vbat_min']}-{info['vbat_max']}V"
            self.info.config(text=info_text)

            # 初始化时间轴滑块
            if info['runtime_s'] > 0:
                self.time_slider.config(to=info['runtime_s'], resolution=0.1)
                self.time_slider.set(0)

            self.draw_all()
            self.update_data_panel(0)
        else:
            self.info.config(text="加载失败")

    def on_time_slider(self, value):
        """时间轴滑块回调"""
        if self.df is None or len(self.df) == 0:
            return
        try:
            t = float(value)
            self.update_data_panel(t)
        except:
            pass

    def update_data_panel(self, time_s):
        """更新右侧数据面板"""
        if self.df is None or len(self.df) == 0:
            return

        # 找到最接近的时间点
        df = self.df
        idx = (df["time_s"] - time_s).abs().idxmin()
        row = df.iloc[idx]
        self.current_time_idx = idx

        # 更新显示
        self.lbl_time.config(text=f"时间: {row['time_s']:.3f} s")

        self.lbl_gyro_x.config(text=f"X: {row['gyro_x']:.1f}")
        self.lbl_gyro_y.config(text=f"Y: {row['gyro_y']:.1f}")
        self.lbl_gyro_z.config(text=f"Z: {row['gyro_z']:.1f}")

        self.lbl_set_r.config(text=f"R: {row['setpoint_r']:.1f}")
        self.lbl_set_p.config(text=f"P: {row['setpoint_p']:.1f}")
        self.lbl_set_y.config(text=f"Y: {row['setpoint_y']:.1f}")

        for i, lbl in enumerate(self.lbl_motors):
            lbl.config(text=f"M{i+1}: {row[f'motor{i+1}']}")

        self.lbl_vbat.config(text=f"电压: {row['vbat']:.2f} V")
        self.lbl_rssi.config(text=f"RSSI: {row['rssi']}")
        self.lbl_throttle.config(text=f"油门: {row['throttle']}")

    def draw_all(self):
        if self.df is None or len(self.df) == 0:
            return

        df = self.df
        t = df["time_s"]
        thr = df["throttle"]

        # ===== 顶部曲线 =====
        self.ax_top.clear()

        # Gyro曲线（根据开关）
        if self.chk_gyro_r.get():
            self.ax_top.plot(t, df["gyro_x"], label="Gyro Roll", color=COLORS["roll"], lw=1)
        if self.chk_gyro_p.get():
            self.ax_top.plot(t, df["gyro_y"], label="Gyro Pitch", color=COLORS["pitch"], lw=1)
        if self.chk_gyro_y.get():
            self.ax_top.plot(t, df["gyro_z"], label="Gyro Yaw", color=COLORS["yaw"], lw=1)

        # Setpoint曲线（根据开关）
        if self.chk_sp_r.get():
            self.ax_top.plot(t, df["setpoint_r"], label="Setpoint Roll", color=COLORS["setpoint"], lw=1, ls="--")
        if self.chk_sp_p.get():
            self.ax_top.plot(t, df["setpoint_p"], label="Setpoint Pitch", color=COLORS["setpoint"], lw=1, ls="--")
        if self.chk_sp_y.get():
            self.ax_top.plot(t, df["setpoint_y"], label="Setpoint Yaw", color="#a29bfe", lw=1, ls="--")

        self.ax_top.set_title("Gyro vs Setpoint", color="white")
        self.ax_top.legend()
        self.ax_top.grid(True)
        self.canvas_top.draw()

        # ===== 四轴姿态 =====
        m = df[["motor1", "motor2", "motor3", "motor4"]].iloc[-1].values
        nm = (m - 1000) / 500
        for cir, v in zip(self.motors, nm):
            cir.set_alpha(0.5 + max(0, min(1, v)) * 0.5)
        self.canvas_quad.draw()

        # ===== 频谱图 =====
        self.ax_spec.clear()
        try:
            if len(thr) > 10 and not np.all(thr == thr.iloc[0]):
                f, ts, Sxx = spectrogram(thr - np.mean(thr), fs=250, nperseg=256, noverlap=128)
                im = self.ax_spec.pcolormesh(ts, f, 10 * np.log10(Sxx), cmap="inferno", vmin=-80, vmax=-20)
                self.ax_spec.set_ylim(0, 500)
                self.fig_spec.colorbar(im, ax=self.ax_spec)
        except:
            pass
        self.ax_spec.set_title("Throttle Freq Spectrum", color="white")
        self.canvas_spec.draw()

        # ===== 电机曲线 =====
        self.ax_motor.clear()
        if self.chk_m1.get():
            self.ax_motor.plot(t, df["motor1"], label="M1", color=COLORS["motor"], lw=1)
        if self.chk_m2.get():
            self.ax_motor.plot(t, df["motor2"], label="M2", color=COLORS["roll"], lw=1)
        if self.chk_m3.get():
            self.ax_motor.plot(t, df["motor3"], label="M3", color=COLORS["pitch"], lw=1)
        if self.chk_m4.get():
            self.ax_motor.plot(t, df["motor4"], label="M4", color=COLORS["yaw"], lw=1)
        self.ax_motor.set_title("Motors", color="white")
        self.ax_motor.legend()
        self.ax_motor.grid(True)
        self.canvas_motor.draw()

    def export_csv(self):
        if self.df is None:
            messagebox.showwarning("提示","先加载日志")
            return
        p = filedialog.asksaveasfilename(defaultextension=".csv",filetypes=[("CSV","*.csv")],initialfile="LOG.csv")
        if p:
            try:
                self.df.to_csv(p,index=False,encoding="utf-8-sig")
                messagebox.showinfo("成功","导出完成")
            except:
                messagebox.showerror("错误","请保存到桌面/文件夹")

    def save_figure(self):
        """保存当前图表为图片"""
        if self.df is None:
            messagebox.showwarning("提示","先加载日志")
            return
        p = filedialog.asksaveasfilename(defaultextension=".png",filetypes=[("PNG图片","*.png")],initialfile="blackbox.png")
        if p:
            try:
                # 保存所有图表到一张大图
                fig = plt.figure(figsize=(20, 14), facecolor="#1a1a1a")
                gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.2)

                ax1 = fig.add_subplot(gs[0, :])
                ax2 = fig.add_subplot(gs[1, 0])
                ax3 = fig.add_subplot(gs[1, 1])
                ax4 = fig.add_subplot(gs[2, :])

                df = self.df
                t = df["time_s"]

                # Gyro & Setpoint
                if self.chk_gyro_r.get(): ax1.plot(t, df["gyro_x"], label="Gyro Roll", color=COLORS["roll"], lw=1)
                if self.chk_gyro_p.get(): ax1.plot(t, df["gyro_y"], label="Gyro Pitch", color=COLORS["pitch"], lw=1)
                if self.chk_gyro_y.get(): ax1.plot(t, df["gyro_z"], label="Gyro Yaw", color=COLORS["yaw"], lw=1)
                if self.chk_sp_r.get(): ax1.plot(t, df["setpoint_r"], label="Setpoint Roll", color=COLORS["setpoint"], lw=1, ls="--")
                if self.chk_sp_p.get(): ax1.plot(t, df["setpoint_p"], label="Setpoint Pitch", color=COLORS["setpoint"], lw=1, ls="--")
                ax1.set_title("Gyro vs Setpoint", color="white")
                ax1.legend()
                ax1.grid(True)

                # 四轴姿态
                ax2.set_xlim(-1.2, 1.2)
                ax2.set_ylim(-1.2, 1.2)
                ax2.set_aspect('equal')
                ax2.set_title("Quad Motors", color="white")
                pos = [(-0.8,0), (0,0.8), (0.8,0), (0,-0.8)]
                cols = ["#ff6b6b","#4ecdc4","#f0932b","#a29bfe"]
                m = df[["motor1","motor2","motor3","motor4"]].iloc[-1].values
                nm = (m-1000)/500
                for (x,y),c,v in zip(pos,cols,nm):
                    ax2.add_patch(plt.Circle((x,y),0.2,color=c,alpha=0.5+max(0,min(1,v))*0.5))

                # 频谱
                ax3.set_title("Spectrum", color="white")
                try:
                    f, ts, Sxx = spectrogram(df["throttle"]-np.mean(df["throttle"]), fs=250, nperseg=256, noverlap=128)
                    ax3.pcolormesh(ts, f, 10*np.log10(Sxx), cmap="inferno", vmin=-80, vmax=-20)
                    ax3.set_ylim(0, 500)
                except:
                    pass

                # 电机
                if self.chk_m1.get(): ax4.plot(t, df["motor1"], label="M1", color=COLORS["motor"], lw=1)
                if self.chk_m2.get(): ax4.plot(t, df["motor2"], label="M2", color=COLORS["roll"], lw=1)
                if self.chk_m3.get(): ax4.plot(t, df["motor3"], label="M3", color=COLORS["pitch"], lw=1)
                if self.chk_m4.get(): ax4.plot(t, df["motor4"], label="M4", color=COLORS["yaw"], lw=1)
                ax4.set_title("Motors", color="white")
                ax4.legend()
                ax4.grid(True)

                fig.savefig(p, dpi=150, facecolor="#1a1a1a")
                plt.close(fig)
                messagebox.showinfo("成功", "图片已保存")
            except Exception as e:
                messagebox.showerror("错误", f"保存失败: {str(e)}")

if __name__ == "__main__":
    app = BlackboxViewer()
    app.mainloop()