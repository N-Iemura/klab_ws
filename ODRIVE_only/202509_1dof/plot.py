import pandas as pd
import matplotlib.pyplot as plt
import os
from matplotlib import rcParams

# フォントをTimesに設定し、サイズを18ptに
rcParams['font.family'] = 'Times New Roman'
rcParams['font.size'] = 18

# CSVファイルを読み込む
csv_file = "csv/ve0_20250924_2157.csv"  # CSVファイルのパスを指定してください
data = pd.read_csv(csv_file)

# データを取得
time = data['time'].to_numpy()
velocity_0 = data['Velocity_0'].to_numpy()
velocity_1 = -data['Velocity_1'].to_numpy()
output_vel_0 = -data['Output_vel_0'].to_numpy()

# 個別プロット
fig, axes = plt.subplots(1, 3, figsize=(15, 5))  # 横に3つのサブプロットを作成

# Velocity_0のプロット
axes[0].plot(time, velocity_0, color='blue')
axes[0].set_xlabel('Time [s]')
axes[0].set_ylabel('Motor_1 [turn/s]')
axes[0].set_xlim(left=0)
axes[0].tick_params(axis='both', direction='in')  # ちょんちょん（目盛）を内向きに

# Velocity_1のプロット
axes[1].plot(time, velocity_1, color='green')
axes[1].set_xlabel('Time [s]')
axes[1].set_ylabel('Motor_2 [turn/s]')
axes[1].set_xlim(left=0)
axes[1].tick_params(axis='both', direction='in')  # ちょんちょん（目盛）を内向きに

# Output_vel_0のプロット
axes[2].plot(time, output_vel_0, color='red')
axes[2].set_xlabel('Time [s]')
axes[2].set_ylabel('Output [turn/s]')
axes[2].set_xlim(left=0)
axes[2].tick_params(axis='both', direction='in')  # ちょんちょん（目盛）を内向きに

# レイアウト調整
plt.tight_layout()

# グラフを保存
output_dir = "fig"
os.makedirs(output_dir, exist_ok=True)  # フォルダが存在しない場合は作成

# 個別プロットの保存
csv_filename = os.path.basename(csv_file)  # ファイル名を取得
plot_filename = os.path.splitext(csv_filename)[0] + "_individual.png"  # 個別プロット用の名前
output_path = os.path.join(output_dir, plot_filename)
plt.savefig(output_path)

plt.show()

# すべてを1つのグラフにまとめたプロット
plt.figure(figsize=(10, 6))
plt.plot(time, velocity_0, label='Motor_1', color='blue')
plt.plot(time, velocity_1, label='Motor_2', color='green')
plt.plot(time, output_vel_0, label='Output', color='red')
plt.xlabel('Time [s]')
plt.ylabel('Values [turn/s]')
plt.legend()
plt.xlim(left=0)
plt.tick_params(axis='both', direction='in')  # ちょんちょん（目盛）を内向きに

# まとめたグラフの保存
combined_plot_filename = os.path.splitext(csv_filename)[0] + "_combined.png"  # まとめたプロット用の名前
combined_output_path = os.path.join(output_dir, combined_plot_filename)
plt.savefig(combined_output_path)

plt.show()