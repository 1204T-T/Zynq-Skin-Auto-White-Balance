import cv2
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import matplotlib
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ================= 解决乱码 =================
matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial Unicode MS']
matplotlib.rcParams['axes.unicode_minus'] = False

cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)  # 尝试请求 30 帧

gain_history = deque(maxlen=50)
kelvin_history = deque(maxlen=50)
zoom_factor = 1.0
pan_x = 0

# 全局状态变量
smooth_gain = 1.0
smooth_kelvin = 5500
use_clahe = False  # 默认关闭画质增强，保证流畅

# 初始化 CLAHE
clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

root = tk.Tk()
root.title("肤色自适应白平衡监控系统")
root.geometry("1200x750")
root.configure(bg="#f4f4f4")

FONT_LARGE = ("Microsoft YaHei", 14, "bold")
FONT_BTN = ("Microsoft YaHei", 12)

toolbar_frame = tk.Frame(root, bg="#e0e0e0", pady=10)
toolbar_frame.pack(side=tk.TOP, fill=tk.X, padx=20, pady=10)


# 事件函数
def exit_app():
    root.quit()
    root.destroy()


def save_img():
    # 保存时截取当前的渲染帧
    plt.savefig("screenshot_result.png", bbox_inches='tight')
    print("截图已保存为 screenshot_result.png")


def zoom_in():
    global zoom_factor
    zoom_factor += 0.2
    if zoom_factor > 3.0: zoom_factor = 3.0


def zoom_out():
    global zoom_factor
    zoom_factor -= 0.2
    if zoom_factor < 1.0: zoom_factor = 1.0


def pan_left():
    global pan_x
    pan_x -= 20


def pan_right():
    global pan_x
    pan_x += 20


def toggle_clahe():
    global use_clahe
    use_clahe = not use_clahe
    btn_clahe.config(text=f"✨ 增强: {'开启' if use_clahe else '关闭'}")


# 按钮布局
tk.Button(toolbar_frame, text="🏠 退出系统", command=exit_app, font=FONT_BTN, width=10, pady=5).pack(side=tk.LEFT,
                                                                                                    padx=10)
tk.Button(toolbar_frame, text="💾 保存截图", command=save_img, font=FONT_BTN, width=10, pady=5).pack(side=tk.LEFT,
                                                                                                    padx=10)
btn_clahe = tk.Button(toolbar_frame, text="✨ 增强: 关闭", command=toggle_clahe, font=FONT_BTN, width=10, pady=5)
btn_clahe.pack(side=tk.LEFT, padx=10)
tk.Button(toolbar_frame, text="🔍 放大 (+)", command=zoom_in, font=FONT_BTN, width=10, pady=5).pack(side=tk.LEFT,
                                                                                                   padx=10)
tk.Button(toolbar_frame, text="🔍 缩小 (-)", command=zoom_out, font=FONT_BTN, width=10, pady=5).pack(side=tk.LEFT,
                                                                                                    padx=10)
tk.Button(toolbar_frame, text="⬅️ 左移", command=pan_left, font=FONT_BTN, width=10, pady=5).pack(side=tk.LEFT, padx=10)
tk.Button(toolbar_frame, text="➡️ 右移", command=pan_right, font=FONT_BTN, width=10, pady=5).pack(side=tk.LEFT, padx=10)

# ================= 主数据展示区 =================
main_frame = tk.Frame(root, bg="#f4f4f4")
main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)

fig = plt.figure(figsize=(11, 5.5))
gs = fig.add_gridspec(1, 2, width_ratios=[1, 1])
ax1 = fig.add_subplot(gs[0, 0])
ax2 = fig.add_subplot(gs[0, 1])
# 初始化双轴
ax2_twin = ax2.twinx()

canvas = FigureCanvasTkAgg(fig, master=main_frame)
canvas_widget = canvas.get_tk_widget()
canvas_widget.grid(row=0, column=0, columnspan=2, sticky="nsew")

label_ratio = tk.Label(main_frame, text="肤色占比: 0.00%", font=FONT_LARGE, fg="#003366", bg="#e6f2ff", pady=10,
                       relief="ridge", bd=2)
label_ratio.grid(row=1, column=0, sticky="ew", padx=20, pady=10)

label_gain = tk.Label(main_frame, text="动态R增益: 1.00 | 实时色温: 5500K", font=FONT_LARGE, fg="#990000", bg="#ffe6e6",
                      pady=10, relief="ridge", bd=2)
label_gain.grid(row=1, column=1, sticky="ew", padx=20, pady=10)

main_frame.grid_rowconfigure(0, weight=1)
main_frame.grid_columnconfigure(0, weight=1)
main_frame.grid_columnconfigure(1, weight=1)

# 提前初始化画图对象，避免每一帧都 clear 重绘
line_gain, = ax2.plot([], [], color='red', linewidth=2, label="R增益")
line_kelvin, = ax2_twin.plot([], [], color='blue', linewidth=2, linestyle='--', label="色温")

ax2_twin.set_ylim(2000, 8000)
ax2_twin.set_ylabel("色温(K)", color='blue', fontsize=12)
ax2_twin.tick_params(axis='y', labelcolor='blue')
ax2.set_title("白平衡动态调整曲线", fontname="Microsoft YaHei", fontsize=14)
ax2.set_ylabel("R增益", color='red', fontsize=12)
ax2.tick_params(axis='y', labelcolor='red')
ax2.set_ylim(0.5, 2.0)
ax2.grid(True)

# 初始化图像对象
im_display = None


def update_frame():
    global zoom_factor, pan_x, smooth_gain, smooth_kelvin, use_clahe, im_display
    ret, frame = cap.read()
    if not ret:
        root.after(10, update_frame)
        return

    # 降采样以减少计算量 (可选)
    frame = cv2.resize(frame, (480, 360))

    # --- 增强处理 ---
    if use_clahe:
        img_ycrcb = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
        y, cr, cb = cv2.split(img_ycrcb)
        y = clahe.apply(y)
        output_frame = cv2.cvtColor(cv2.merge([y, cr, cb]), cv2.COLOR_YCrCb2BGR)
    else:
        output_frame = frame.copy()
        img_ycrcb = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
        y, cr, cb = cv2.split(img_ycrcb)

    # --- 肤色检测逻辑 ---
    skin_mask_ycrcb = ((cr > 133) & (cr < 173) & (cb > 77) & (cb < 127)).astype(np.uint8) * 255
    img_hls = cv2.cvtColor(output_frame, cv2.COLOR_BGR2HLS)
    h, l, s = cv2.split(img_hls)
    skin_mask_hls = ((h >= 0) & (h <= 25) & (s >= 40) & (s <= 200)).astype(np.uint8) * 255
    skin_mask = cv2.bitwise_and(skin_mask_ycrcb, skin_mask_hls)

    skin_ratio = np.sum(skin_mask == 255) / (frame.shape[0] * frame.shape[1])
    target_gain = 1.0

    if skin_ratio > 0.01:
        kernel = np.ones((5, 5), np.uint8)
        skin_mask = cv2.morphologyEx(skin_mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(skin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid_skin = False
        for cnt in contours:
            area = cv2.contourArea(cnt)
            x, y, w, h = cv2.boundingRect(cnt)
            if area > 2000 and 0.3 < float(w) / h < 2.5:
                cv2.rectangle(output_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                valid_skin = True

        if valid_skin:
            skin_g = np.mean(output_frame[:, :, 1][skin_mask == 255])
            skin_r = np.mean(output_frame[:, :, 2][skin_mask == 255])
            target_gain = np.clip(1.3 / (skin_r / (skin_g + 1e-5)), 0.8, 1.5)

    # 平滑处理
    if skin_ratio > 0.01:
        smooth_gain = smooth_gain * 0.9 + target_gain * 0.1
    smooth_kelvin = int(np.clip(8000 - (smooth_gain - 0.8) * (5500 / 0.7), 2000, 8000))

    gain_history.append(smooth_gain)
    kelvin_history.append(smooth_kelvin)

    # --- 极速绘图渲染 ---
    # 使用 set_data 更新曲线，而不是清空重绘
    x_data = list(range(len(gain_history)))
    line_gain.set_xdata(x_data)
    line_gain.set_ydata(gain_history)

    line_kelvin.set_xdata(x_data)
    line_kelvin.set_ydata(kelvin_history)

    ax2.set_xlim(0, 50)

    # 使用 set_data 更新图像，避免 ax1.clear()
    rgb_frame = cv2.cvtColor(output_frame, cv2.COLOR_BGR2RGB)
    if im_display is None:
        im_display = ax1.imshow(rgb_frame)
        ax1.axis('off')
        ax1.set_title("实时监控画面", fontname="Microsoft YaHei", fontsize=14)
        # 锁定显示范围
        h_img, w_img, _ = rgb_frame.shape
        ax1.set_xlim(0, w_img)
        ax1.set_ylim(h_img, 0)
    else:
        im_display.set_data(rgb_frame)

    # 处理缩放平移
    h_img, w_img, _ = rgb_frame.shape
    center_x = w_img / 2 + pan_x
    view_w = w_img / zoom_factor
    ax1.set_xlim(max(0, center_x - view_w / 2), min(w_img, center_x + view_w / 2))

    # 刷新画布
    canvas.draw_idle()  # draw_idle 比 draw 效率高得多

    # 更新文字
    label_ratio.config(text=f"肤色占比: {skin_ratio * 100:5.2f}%")
    label_gain.config(text=f"动态R增益: {smooth_gain:4.2f}  |  实时色温: {smooth_kelvin}K")

    # 循环刷新，缩短等待时间以提高帧率
    root.after(10, update_frame)


root.after(100, update_frame)
root.mainloop()
cap.release()