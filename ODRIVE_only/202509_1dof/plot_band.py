import pandas as pd
import matplotlib.pyplot as plt
import os
from matplotlib import rcParams

# フォントをTimesに設定
rcParams['font.family'] = 'Times New Roman'

# CSVファイルを読み込む
csv_file = "csv/two_pos_trac_20250801_1947.csv"  # CSVファイルのパスを指定してください
data = pd.read_csv(csv_file)

# 10秒から30秒のデータを抽出
filtered_data = data[(data['time'] >= 10) & (data['time'] <= 30)].copy()
filtered_data['time'] -= 10  # 時間を0秒から始まるように調整

# データを取得
time = filtered_data['time']
velocity_0 = filtered_data['Velocity_0']
velocity_1 = -filtered_data['Velocity_1']
output_vel_0 = -filtered_data['Output_vel_0']

# 個別プロット
fig, axes = plt.subplots(1, 3, figsize=(15, 5))  # 横に3つのサブプロットを作成

# Velocity_0のプロット
axes[0].plot(time, velocity_0, color='blue')
axes[0].set_xlabel('Time [s]')  # 単位を追加
axes[0].set_ylabel('Velocity_0 [turn/s]')  # 単位を追加

# Velocity_1のプロット
axes[1].plot(time, velocity_1, color='green')
axes[1].set_xlabel('Time [s]')  # 単位を追加
axes[1].set_ylabel('Velocity_1 [turn/s]')  # 単位を追加

# Output_vel_0のプロット
axes[2].plot(time, output_vel_0, color='red')
axes[2].set_xlabel('Time [s]')  # 単位を追加
axes[2].set_ylabel('Output_vel [turn/s]')  # 単位を追加

# レイアウト調整
plt.tight_layout()

# グラフを保存
output_dir = "fig"
os.makedirs(output_dir, exist_ok=True)  # フォルダが存在しない場合は作成

# 個別プロットの保存
csv_filename = os.path.basename(csv_file)  # ファイル名を取得
plot_filename = os.path.splitext(csv_filename)[0] + "_individual_10_30s.png"  # 個別プロット用の名前
output_path = os.path.join(output_dir, plot_filename)
plt.savefig(output_path)

plt.show()

# すべてを1つのグラフにまとめたプロット
plt.figure(figsize=(10, 6))
plt.plot(time, velocity_0, label='Velocity_0', color='blue')
plt.plot(time, velocity_1, label='Velocity_1', color='green')
plt.plot(time, output_vel_0, label='Output_vel', color='red')
plt.xlabel('Time [s]')  # 単位を追加
plt.ylabel('Values [turn/s]')  # 単位を追加
plt.legend()

# まとめたグラフの保存
combined_plot_filename = os.path.splitext(csv_filename)[0] + "_combined_10_30s.png"  # まとめたプロット用の名前
combined_output_path = os.path.join(output_dir, combined_plot_filename)
plt.savefig(combined_output_path)

plt.show()