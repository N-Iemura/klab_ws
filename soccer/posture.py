import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter

# フォントをTimesに設定
plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams["font.size"] = 14

l1 = 0.45
l2 = 0.5
dt = 0.01
m = 0.45
v = 1
fx = m * v / dt

a_values = np.linspace(-np.pi/2, np.pi/2, 1000)
b_values = np.linspace(-np.pi/2, np.pi/2, 1000)

# y=-0.7のときのxの範囲を求める
fixed_y = -0.7
x_values = []
a_vals = []
b_vals = []
torque_sums = []
for a in a_values:
    for b in b_values:
        if 0 < a + b < 2 * np.pi:  # 条件を追加
            y = -l1 * np.cos(a) - l2 * np.cos(b)
            if np.isclose(y, fixed_y, atol=1e-3):  # yが固定値に近い場合
                x = l1 * np.sin(a) - l2 * np.sin(b)
                x_values.append(x)
                a_vals.append(a)
                b_vals.append(b)

# new_torqueの時間変化をプロット
t_list = np.linspace(0, 0.1, num=100)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

for i, (a, b) in enumerate(zip(a_vals, b_vals)):
    new_torque1_list = []
    new_torque2_list = []
    torque_sum_list = []
    for t in t_list:
        if 0 <= t <= 0.05:
            fx = 0
            new_torque1 = 0
            new_torque2 = 0
        elif 0.05 < t < 0.06:
            fx = m * v / dt
            new_torque1 = l1 * np.cos(a) * fx
            new_torque2 = -l2 * np.cos(b) * fx
        elif t >= 0.06:
            fx = 0
            new_torque1 = 0
            new_torque2 = 0

        new_torque1_list.append(new_torque1)
        new_torque2_list.append(new_torque2)
        torque_sum_list.append(new_torque1 + new_torque2)

    ax1.plot(t_list, new_torque1_list, color=plt.cm.viridis(i / len(a_vals)))
    ax2.plot(t_list, new_torque2_list, color=plt.cm.viridis(i / len(a_vals)))
    torque_sums.append(np.sum(torque_sum_list))

# トルクの和が最小となるxの値を求める
min_torque_sum_index = np.argmin(torque_sums)
min_torque_sum_x = x_values[min_torque_sum_index]

# 軸ラベルとタイトル
ax1.set_ylabel('Torque 1 [Nm]')
ax1.yaxis.set_major_formatter(FormatStrFormatter('%.0f'))
ax2.yaxis.set_major_formatter(FormatStrFormatter('%.0f'))
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Torque 2 [Nm]')

# カラーバーを全体の右側に配置
fig.subplots_adjust(right=0.85, hspace=0.3)  # サブプロットの間隔調整
cbar_ax = fig.add_axes([0.9, 0.15, 0.03, 0.7])
sm = plt.cm.ScalarMappable(cmap='viridis', norm=plt.Normalize(vmin=min(x_values), vmax=max(x_values)))
sm.set_array([])
cbar = fig.colorbar(sm, cax=cbar_ax)
cbar.set_label(r'$\it{x}$ values', labelpad=10, rotation=0, ha='center')

plt.tight_layout(rect=[0, 0, 0.85, 1])  # 全体レイアウト調整
plt.show()

print(f"トルクの和が最小となるxの値: {min_torque_sum_x}")