import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re

# --- 配置 ---
SERIAL_PORT = 'COM14'
BAUD_RATE = 115200
# ----------

# 禁用硬件流控，防止 ESP32 触发复位
ser = serial.Serial(
    port=SERIAL_PORT,
    baudrate=BAUD_RATE,
    timeout=0.1,
    dsrdtr=False,
    rtscts=False
)

# 用于存储数据的列表
x_vals, y_vals, z_vals = [], [], []
max_points = 100 

# 设置绘图区域
fig, ax = plt.subplots()
line_x, = ax.plot([], [], label='Mag X', color='red')
line_y, = ax.plot([], [], label='Mag Y', color='green')
line_z, = ax.plot([], [], label='Mag Z', color='blue')

ax.set_ylim(-500, 500) # 这里根据你的传感器数值范围调整
ax.legend(loc='upper right')

# 正则表达式：查找三个连续的浮点数（中间可能包含逗号或空格）
# 它可以跳过前面的任何文本，只提取数字
pattern = re.compile(r'([-+]?\d*\.\d+|\d+)[,\s]+([-+]?\d*\.\d+|\d+)[,\s]+([-+]?\d*\.\d+|\d+)')

def update(frame):
    if ser.in_waiting:
        try:
            # 读取一行数据并解码
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            # 搜索匹配模式
            match = pattern.search(line)
            if match:
                # 提取提取到的三个数字
                vals = [float(match.group(1)), float(match.group(2)), float(match.group(3))]
                
                x_vals.append(vals[0])
                y_vals.append(vals[1])
                z_vals.append(vals[2])
                
                # 保持列表长度
                if len(x_vals) > max_points:
                    x_vals.pop(0); y_vals.pop(0); z_vals.pop(0)
                
                # 更新曲线数据
                line_x.set_data(range(len(x_vals)), x_vals)
                line_y.set_data(range(len(y_vals)), y_vals)
                line_z.set_data(range(len(z_vals)), z_vals)
                ax.set_xlim(0, len(x_vals))
                
        except Exception as e:
            # 这里的异常会被静默处理，保证绘图不崩溃
            pass
    return line_x, line_y, line_z

# 启动动画
ani = FuncAnimation(fig, update, interval=20, blit=True)
plt.show()