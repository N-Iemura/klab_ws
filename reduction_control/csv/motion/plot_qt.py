import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
from scipy.optimize import minimize

# Set font properties globally
plt.rcParams['font.size'] = 20
plt.rcParams['font.family'] = 'Times New Roman'

# CSVファイルを読み込む
df = pd.read_csv('csv/motion/two_pos_trac_20250218_111129.csv', skiprows=2, usecols=[0, 1, 3, 4, 6, 7, 8], names=['time', 'Ref_0', 'Position_0', 'Ref_1', 'Position_1', 'Current_0', 'Current_1'])

filtered_df = df[(df['time'] >= 0) & (df['time'] <= 20)]

# 時間と速度を取得
time = filtered_df['time'].to_numpy()
speed_0 = np.gradient(filtered_df['Position_0'].to_numpy(), time)
speed_1 = np.gradient(filtered_df['Position_1'].to_numpy(), time)

v_max = 0.5
t_max = 10  # 仮の最大トルク値

# 損失関数の定義
def loss_function(reduction_ratios, speed, torque, v_max, t_max):
    # 速度偏差、トルク消費、効率を考慮した損失関数
    speed_loss = np.sum((speed - v_max)**2)
    torque_loss = np.sum((torque / t_max)**2)
    efficiency_loss = np.sum(1 / reduction_ratios)  # 仮の効率モデル
    return speed_loss + torque_loss + efficiency_loss

# 最適化の初期値
initial_reduction_ratios = np.full_like(speed_0, 33)

# 制約条件（減速比の範囲）
bounds = [(7, 33) for _ in range(len(speed_0))]

# 最適化を実行
result_0 = minimize(loss_function, initial_reduction_ratios, args=(speed_0, filtered_df['Current_0'].to_numpy(), v_max, t_max), bounds=bounds)
result_1 = minimize(loss_function, initial_reduction_ratios, args=(speed_1, filtered_df['Current_1'].to_numpy(), v_max, t_max), bounds=bounds)

# 最適化された減速比
optimized_reduction_ratio_0 = result_0.x
optimized_reduction_ratio_1 = result_1.x

# グラフを描画するためのサブプロットを作成する
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 10))

# 1つ目のグラフ: Referenceのみ
ax1.plot(time, -360 * filtered_df['Ref_0'].to_numpy(), label='Ref_1')
ax1.plot(time, -360 * filtered_df['Ref_1'].to_numpy(), label='Ref_2')
ax1.legend(loc='upper left')
ax1.set_ylabel('Reference [deg]')

# 2つ目のグラフ: Reduction Ratio
ax2.plot(time, optimized_reduction_ratio_0, label='Optimized Reduction Ratio 0', linestyle='--')
ax2.plot(time, optimized_reduction_ratio_1, label='Optimized Reduction Ratio 1', linestyle='--')
ax2.legend(loc='lower left')
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Reduction Ratio')

# 最適化された減速比をデータフレームに追加
filtered_df['Optimized_Reduction_Ratio_0'] = optimized_reduction_ratio_0
filtered_df['Optimized_Reduction_Ratio_1'] = optimized_reduction_ratio_1

# 現在の日時を取得してファイル名に追加
current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
csv_filename = f'csv/motion/two_pos_optimized_red.csv'
svg_filename = f'fig/optimized_trac.svg'

# csv_filename = f'csv/motion/two_pos_optimized_red_{current_time}.csv'
# svg_filename = f'fig/optimized_trac_{current_time}.svg'


# 新しいCSVファイルとして保存
filtered_df.to_csv(csv_filename, index=False)

# グラフを保存
plt.tight_layout()
plt.savefig(svg_filename, format='svg')

# グラフを表示する
plt.show()