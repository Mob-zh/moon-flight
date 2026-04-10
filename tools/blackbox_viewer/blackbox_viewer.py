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
# ------------------------------------------------------------
# 【待实现 / 可优化功能（按图片效果进一步完善）💡】
# 1. 界面布局精细化：严格对齐 Betaflight 区域比例与间距
# 2. 增加右侧数据面板：实时显示当前帧 Gyro、PID、电机、电压等值
# 3. 增加时间轴滑块：支持拖动查看任意时间段数据
# 4. 增加曲线开关：可单独显示/隐藏 Gyro、Setpoint、Motor 等曲线
# 5. 完善 P 帧差分解码：实现真正的 Betaflight TAG8_4S16 完整解包
# 6. 增加频谱图配色与色标优化，更贴近官方显示效果
# 7. 增加滤波显示：原始 Gyro / 滤波后 Gyro 对比曲线
# 8. 增加 D-term、PID 输出、Mix 混控输出等专业通道显示
# 9. 增加电机联动动画：四轴姿态图随电机输出实时动态变化
# 10. 增加图表区域缩放、框选、平移交互功能
# 11. 增加日志信息显示：采样率、总时长、帧数、黑匣子版本
# 12. 增加异常数据保护：防止越界、溢出、NaN 导致崩溃
# 13. 支持批量日志加载与多日志对比显示
# 14. 支持直接保存图表为图片 / 复制到剪贴板
# 15. 支持噪声统计：RMS、峰值、主要振动频率自动标注
# ------------------------------------------------------------
# 【已知当前限制 📌】
# 1. P 帧解析为简化版本，未完整实现所有压缩格式解码
# 2. 四轴姿态图仅为静态示意，未做实时动态电机强度动画
# 3. 无时间滑块，图表一次性绘制全部数据
# 4. 无曲线显隐开关与交互缩放功能
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
            "time_s": time_s,
            "throttle": throttle,
            "motor1": m0, "motor2": m1, "motor3": m2, "motor4": m3,
            "gyro_x": gyro[0], "gyro_y": gyro[1], "gyro_z": gyro[2],
            "setpoint_r": setpoint[0], "setpoint_p": setpoint[1], "setpoint_y": setpoint[2],
            "vbat": vbat, "rssi": rssi
        }
        self.data.append(frame)
        self.last_i_frame = frame.copy()
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

        self.data.append(last)
        return ptr

# ==================== GUI ====================
class BlackboxViewer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Betaflight Blackbox 风格查看器")
        self.geometry("1600x900")
        self.parser = BlackboxParser()
        self.df = None
        self.configure(bg="#1a1a1a")

        top = ttk.Frame(self, height=60)
        top.pack(fill=tk.X, padx=10, pady=8)

        ttk.Button(top, text="打开日志", command=self.open_log, width=12).pack(side=tk.LEFT, padx=5)
        ttk.Button(top, text="导出CSV", command=self.export_csv, width=12).pack(side=tk.LEFT, padx=5)
        self.info = ttk.Label(top, text="未加载日志", font=("Arial", 11))
        self.info.pack(side=tk.LEFT, padx=20)

        self.main_frame = ttk.Frame(self)
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.fig_top = plt.Figure(figsize=(16, 4), dpi=90)
        self.ax_top = self.fig_top.add_subplot(111)
        self.canvas_top = FigureCanvasTkAgg(self.fig_top, self.main_frame)
        self.canvas_top.get_tk_widget().pack(fill=tk.X, padx=5, pady=5)

        self.mid = ttk.Frame(self.main_frame)
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
        self.canvas_motor = FigureCanvasTkAgg(self.fig_motor, self.main_frame)
        self.canvas_motor.get_tk_widget().pack(fill=tk.X, padx=5, pady=5)

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
            self.info.config(text=f"完成 {len(self.df)} 帧")
            self.draw_all()
        else:
            self.info.config(text="加载失败")

    def draw_all(self):
        df = self.df
        t = df["time_s"]
        thr = df["throttle"]

        self.ax_top.clear()
        self.ax_top.plot(t,df["gyro_x"],label="Gyro Roll",color=COLORS["roll"],lw=1)
        self.ax_top.plot(t,df["setpoint_r"],label="Setpoint Roll",color=COLORS["setpoint"],lw=1,ls="--")
        self.ax_top.plot(t,df["gyro_y"],label="Gyro Pitch",color=COLORS["pitch"],lw=1)
        self.ax_top.plot(t,df["setpoint_p"],label="Setpoint Pitch",color=COLORS["setpoint"],lw=1,ls="--")
        self.ax_top.set_title("Gyro vs Setpoint",color="white")
        self.ax_top.legend()
        self.ax_top.grid(True)
        self.canvas_top.draw()

        m = df[["motor1","motor2","motor3","motor4"]].iloc[-1].values
        nm = (m-1000)/500
        for cir, v in zip(self.motors, nm):
            cir.set_alpha(0.5 + v*0.5)
        self.canvas_quad.draw()

        self.ax_spec.clear()
        f,ts,Sxx = spectrogram(thr-np.mean(thr),fs=250,nperseg=256,noverlap=128)
        im = self.ax_spec.pcolormesh(ts,f,10*np.log10(Sxx),cmap="inferno",vmin=-80,vmax=-20)
        self.ax_spec.set_ylim(0,500)
        self.ax_spec.set_title("Throttle Freq Spectrum",color="white")
        self.fig_spec.colorbar(im,ax=self.ax_spec)
        self.canvas_spec.draw()

        self.ax_motor.clear()
        self.ax_motor.plot(t,df["motor1"],label="M1",color=COLORS["motor"],lw=1)
        self.ax_motor.plot(t,df["motor2"],label="M2",color=COLORS["roll"],lw=1)
        self.ax_motor.plot(t,df["motor3"],label="M3",color=COLORS["pitch"],lw=1)
        self.ax_motor.plot(t,df["motor4"],label="M4",color=COLORS["yaw"],lw=1)
        self.ax_motor.set_title("Motors",color="white")
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

if __name__ == "__main__":
    app = BlackboxViewer()
    app.mainloop()