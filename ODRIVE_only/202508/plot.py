import pandas as pd
import matplotlib.pyplot as plt
import os

# CSVファイルを読み込む
csv_file = "csv/two_pos_trac_20250801_1705.csv"  # CSVファイルのパスを指定してください
data = pd.read_csv(csv_file)

# データを取得
time = data['time']
velocity_0 = data['Velocity_0']
velocity_1 = -data['Velocity_1']
output_vel_0 = -data['Output_vel_0']

# 個別プロット
fig, axes = plt.subplots(1, 3, figsize=(15, 5))  # 横に3つのサブプロットを作成

# Velocity_0のプロット
axes[0].plot(time, velocity_0, color='blue')
axes[0].set_xlabel('Time')
axes[0].set_ylabel('Velocity_0')

# Velocity_1のプロット
axes[1].plot(time, velocity_1, color='green')
axes[1].set_xlabel('Time')
axes[1].set_ylabel('Velocity_1')

# Output_vel_0のプロット
axes[2].plot(time, output_vel_0, color='red')
axes[2].set_xlabel('Time')
axes[2].set_ylabel('Output_vel_0')

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
plt.plot(time, velocity_0, label='Velocity_0', color='blue')
plt.plot(time, velocity_1, label='Velocity_1', color='green')
plt.plot(time, output_vel_0, label='Output_vel_0', color='red')
plt.xlabel('Time')
plt.ylabel('Values')
plt.legend()

# まとめたグラフの保存
combined_plot_filename = os.path.splitext(csv_filename)[0] + "_combined.png"  # まとめたプロット用の名前
combined_output_path = os.path.join(output_dir, combined_plot_filename)
plt.savefig(combined_output_path)

plt.show()