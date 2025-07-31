import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime 

# Set font properties globally
plt.rcParams['font.size'] = 20
plt.rcParams['font.family'] = 'Times New Roman'

# CSVファイルを読み込む
df = pd.read_csv('csv/motion/two_pos_trac_20250218_111129.csv', skiprows=2, usecols=[0, 1, 3, 4, 6, 7, 8], names=['time', 'Ref_0', 'Position_0', 'Ref_1', 'Position_1', 'Current_0', 'Current_1'])

filtered_df = df[(df['time'] >= 5) & (df['time'] <= 15)]

# 時間と速度を取得
time = filtered_df['time'].to_numpy()
speed_0 = np.gradient(filtered_df['Position_0'].to_numpy(), time)
speed_1 = np.gradient(filtered_df['Position_1'].to_numpy(), time)

v_max = 0.5
v_f = 0.12

# 減速比を速度に応じて調整
reduction_ratio_0 = np.where(np.abs(speed_0) > v_f, 7 + (33 - 7) * (v_max - np.abs(speed_0)) / v_max, 33)
reduction_ratio_1 = np.where(np.abs(speed_1) > v_f, 7 + (33 - 7) * (v_max - np.abs(speed_1)) / v_max, 33)

# グラフを描画するためのサブプロットを作成する
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 10))

# 1つ目のグラフ: Referenceのみ
ax1.plot(time, -360 * filtered_df['Ref_0'].to_numpy(), label='Ref_1')
ax1.plot(time, -360 * filtered_df['Ref_1'].to_numpy(), label='Ref_2')
ax1.legend(loc='upper left')
ax1.set_ylabel('Reference [deg]')

# 2つ目のグラフ: Reduction Ratio and Reference
ax2.plot(time, reduction_ratio_0, label='Reduction Ratio 0')
ax2.plot(time, reduction_ratio_1, label='Reduction Ratio 1')
ax2.legend(loc='lower left')
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Reduction Ratio')

# 減速比をデータフレームに追加
filtered_df['Reduction_Ratio_0'] = reduction_ratio_0
filtered_df['Reduction_Ratio_1'] = reduction_ratio_1

# 現在の日時を取得してファイル名に追加
current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
csv_filename = f'csv/motion/two_pos_red_{current_time}.csv'
svg_filename = f'fig/copy_trac_1_vr_{current_time}.svg'

# 新しいCSVファイルとして保存
filtered_df.to_csv(csv_filename, index=False)

# グラフを保存
plt.tight_layout()
plt.savefig(svg_filename, format='svg')

# グラフを表示する
plt.show()