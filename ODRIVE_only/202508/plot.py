import pandas as pd
import matplotlib.pyplot as plt
import os
from matplotlib import rcParams

# フォントをTimesに設定し、サイズを18ptに
rcParams['font.family'] = 'Times New Roman'
rcParams['font.size'] = 18

# CSVファイルを読み込む
csv_file = "csv/ve0_20250807_1517.csv"  # CSVファイルのパスを指定してください
data = pd.read_csv(csv_file)

# データを取得
time_raw = data['time']
time = time_raw - time_raw.iloc[0]  # 時間を0からスタートするように調整
velocity_0 = data['Velocity_0']
velocity_1 = -data['Velocity_1']
output_vel_0 = -data['Output_vel_0']

# 位置データを取得
position_0 = data['Position_0']
position_1 = -data['Position_1']
output_pos_0 = -data['Output_pos_0']

# 速度の個別プロット
fig1, axes1 = plt.subplots(1, 3, figsize=(15, 5))  # 横に3つのサブプロットを作成

# Velocity_0のプロット
axes1[0].plot(time, velocity_0, color='blue')
axes1[0].set_xlabel('Time [s]')  # 単位を追加
axes1[0].set_ylabel('Velocity_0 [turn/s]')  # 単位を追加
axes1[0].set_title('Motor 0 Velocity')
axes1[0].set_xlim(0, time.max())  # x軸の範囲を設定（右端の余白を削除）

# Velocity_1のプロット
axes1[1].plot(time, velocity_1, color='green')
axes1[1].set_xlabel('Time [s]')  # 単位を追加
axes1[1].set_ylabel('Velocity_1 [turn/s]')  # 単位を追加
axes1[1].set_title('Motor 1 Velocity')
axes1[1].set_xlim(0, time.max())  # x軸の範囲を設定（右端の余白を削除）

# Output_vel_0のプロット
axes1[2].plot(time, output_vel_0, color='red')
axes1[2].set_xlabel('Time [s]')  # 単位を追加
axes1[2].set_ylabel('Output_vel [turn/s]')  # 単位を追加
axes1[2].set_title('Output Velocity')
axes1[2].set_xlim(0, time.max())  # x軸の範囲を設定（右端の余白を削除）

# レイアウト調整
plt.tight_layout()

# グラフを保存
output_dir = "fig"
os.makedirs(output_dir, exist_ok=True)  # フォルダが存在しない場合は作成

# 速度個別プロットの保存
csv_filename = os.path.basename(csv_file)  # ファイル名を取得
plot_filename = os.path.splitext(csv_filename)[0] + "_velocity_individual.png"  # 速度個別プロット用の名前
output_path = os.path.join(output_dir, plot_filename)
plt.savefig(output_path)

plt.show()

# 位置の個別プロット
fig2, axes2 = plt.subplots(1, 3, figsize=(15, 5))  # 横に3つのサブプロットを作成

# Position_0のプロット
axes2[0].plot(time, position_0, color='blue')
axes2[0].set_xlabel('Time [s]')  # 単位を追加
axes2[0].set_ylabel('Position_0 [turn]')  # 単位を追加
axes2[0].set_title('Motor 0 Position')
axes2[0].set_xlim(0, time.max())  # x軸の範囲を設定（右端の余白を削除）

# Position_1のプロット
axes2[1].plot(time, position_1, color='green')
axes2[1].set_xlabel('Time [s]')  # 単位を追加
axes2[1].set_ylabel('Position_1 [turn]')  # 単位を追加
axes2[1].set_title('Motor 1 Position')
axes2[1].set_xlim(0, time.max())  # x軸の範囲を設定（右端の余白を削除）

# Output_pos_0のプロット
axes2[2].plot(time, output_pos_0, color='red')
axes2[2].set_xlabel('Time [s]')  # 単位を追加
axes2[2].set_ylabel('Output_pos [turn]')  # 単位を追加
axes2[2].set_title('Output Position')
axes2[2].set_xlim(0, time.max())  # x軸の範囲を設定（右端の余白を削除）

# レイアウト調整
plt.tight_layout()

# 位置個別プロットの保存
position_plot_filename = os.path.splitext(csv_filename)[0] + "_position_individual.png"  # 位置個別プロット用の名前
position_output_path = os.path.join(output_dir, position_plot_filename)
plt.savefig(position_output_path)

plt.show()

# すべての速度を1つのグラフにまとめたプロット
plt.figure(figsize=(10, 6))
plt.plot(time, velocity_0, label='Velocity_0', color='blue')
plt.plot(time, velocity_1, label='Velocity_1', color='green')
plt.plot(time, output_vel_0, label='Output_vel', color='red')
plt.xlabel('Time [s]')  # 単位を追加
plt.ylabel('Velocity [turn/s]')  # 単位を追加
plt.title('All Velocities Combined')
plt.xlim(0, time.max())  # x軸の範囲を設定（右端の余白を削除）
plt.legend()

# まとめた速度グラフの保存
velocity_combined_plot_filename = os.path.splitext(csv_filename)[0] + "_velocity_combined.png"  # まとめた速度プロット用の名前
velocity_combined_output_path = os.path.join(output_dir, velocity_combined_plot_filename)
plt.savefig(velocity_combined_output_path)

plt.show()

# すべての位置を1つのグラフにまとめたプロット
plt.figure(figsize=(10, 6))
plt.plot(time, position_0, label='Position_0', color='blue')
plt.plot(time, position_1, label='Position_1', color='green')
plt.plot(time, output_pos_0, label='Output_pos', color='red')
plt.xlabel('Time [s]')  # 単位を追加
plt.ylabel('Position [turn]')  # 単位を追加
plt.title('All Positions Combined')
plt.xlim(0, time.max())  # x軸の範囲を設定（右端の余白を削除）
plt.legend()
plt.xlim(left=0)
plt.tick_params(axis='both', direction='in')  # ちょんちょん（目盛）を内向きに

# まとめた位置グラフの保存
position_combined_plot_filename = os.path.splitext(csv_filename)[0] + "_position_combined.png"  # まとめた位置プロット用の名前
position_combined_output_path = os.path.join(output_dir, position_combined_plot_filename)
plt.savefig(position_combined_output_path)

plt.show()