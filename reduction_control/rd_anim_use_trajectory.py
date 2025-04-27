import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

# Torque values (approximate x-axis range)
torque = np.linspace(0, 11, 50)

# Speed (RPM) - Concave-down quadratic approximation
vertex = 15
a = (3800 - 2300) / ((0 - vertex)**2 - (12 - vertex)**2)
b = -2 * a * vertex
c = 3800
speed = a * torque**2 + b * torque + c

# Reduction ratios to animate
ratios = np.linspace(7, 33, 100)

# Set up the figure and axis
fig, ax = plt.subplots(figsize=(8, 6))
line, = ax.plot([], [], color='blue', lw=2)
fill = None

# Set axis labels and limits
font_properties = {'family': 'Times New Roman', 'size': 18}
ax.set_xlabel('Torque [Nm]', fontdict=font_properties)
ax.set_ylabel('Speed [rpm]', fontdict=font_properties)
ax.set_xlim(0, 380)
ax.set_ylim(0, 580)
ax.tick_params(labelsize=16)

# Initialization function for the animation
def init():
    global fill
    line.set_data([], [])
    if fill is not None:  # 修正: fill が None でない場合のみ削除
        try:
            fill.remove()
        except ValueError:
            pass  # fill がリストに存在しない場合は無視
    return line,

def update(frame):
    global fill
    ratio = ratios[frame]
    reduced_speed = speed / ratio
    torque_scaled = torque * ratio
    line.set_data(torque_scaled, reduced_speed)
    if fill is not None:  # 修正: fill が None でない場合のみ削除
        try:
            fill.remove()
        except ValueError:
            pass  # fill がリストに存在しない場合は無視
    fill = ax.fill_between(torque_scaled, reduced_speed, color='blue', alpha=0.3)
    ax.set_title(f"Reduction Ratio: {ratio:.2f}", fontsize=18, family='Times New Roman')
    return line, fill

# Create the animation
ani = FuncAnimation(fig, update, frames=len(ratios), init_func=init, blit=False, interval=100)

# Save the animation as a video or GIF (optional)
ani.save('ani/rd33_to_7.gif', writer='imagemagick')

# Show the animation
plt.show()