import numpy as np
import matplotlib.pyplot as plt

# 定数と初期条件の定義
l1 = 0.5
l2 = 0.5
dt = 0.01
m = 0.45

desired_pos0 = 0.5
desired_pos1 = 0.7

t_list = np.linspace(0, 0.1, num=100)
v = 1

# xの範囲を設定
x_values = np.linspace(-0.1, 0.1, num=5)

# トルク値を保存するリスト
torque0_results = []
torque1_results = []

# xの各値に対してシミュレーションを実行
for x in x_values:
    fx_list = []
    torque0_list = []
    torque1_list = []
    
    # シミュレーションループ
    for t in t_list:
        # 初期化
        new_torque0 = 0
        new_torque1 = 0
        
        if 0 < t <= 0.05:
            fx = 0
        elif 0.05 < t < 0.06:
            fx = m * v / dt
            desired_pos0 = np.arcsin((x + l2 * np.sin(desired_pos1)) / l1)
            desired_pos1 = np.arccos((0.7 - l1 * np.cos(desired_pos0)) / l2)
            new_torque0 = l1 * np.cos(desired_pos0) * fx
            new_torque1 = -(0.7 + l1 * np.cos(desired_pos0)) / l2 * fx
        elif t >= 0.06:
            fx = 0
        
        torque0_list.append(new_torque0)
        torque1_list.append(new_torque1)
    
    torque0_results.append(torque0_list)
    torque1_results.append(torque1_list)

# トルクの時間変化をプロット
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

for i, x in enumerate(x_values):
    ax1.plot(t_list, torque0_results[i], label=f'x={x:.2f}')
    ax2.plot(t_list, torque1_results[i], label=f'x={x:.2f}')

ax1.set_ylabel('Torque 0 [Nm]')
ax1.legend(loc='upper right')
ax1.set_title('Torque 0 over Time for different x values')

ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Torque 1 [Nm]')
ax2.legend(loc='upper right')
ax2.set_title('Torque 1 over Time for different x values')

plt.tight_layout()
plt.show()