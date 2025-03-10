import pandas as pd
import matplotlib.pyplot as plt

# CSVファイルを読み込む
df = pd.read_csv('csv/two_pos_trac_20250218_111129.csv', skiprows=2, usecols=[0, 1, 3, 4, 6, 7, 8], names=['time', 'Ref_0', 'Position_0', 'Ref_1', 'Position_1', 'Current_0', 'Current_1'])

filtered_df = df[(df['time'] >= 0) & (df['time'] <= 20)]

# グラフを描画するためのサブプロットを作成する
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# 1つ目のグラフ: PositionとReference
ax1.plot(filtered_df['time'].to_numpy(), -360*filtered_df['Position_0'].to_numpy(), label='theta_1')
ax1.plot(filtered_df['time'].to_numpy(), -360*filtered_df['Position_1'].to_numpy(), label='theta_2')
ax1.plot(filtered_df['time'].to_numpy(), -360*filtered_df['Ref_0'].to_numpy(), label='Ref_1', linestyle='--')
ax1.plot(filtered_df['time'].to_numpy(), -360*filtered_df['Ref_1'].to_numpy(), label='Ref_2', linestyle='--')
ax1.legend(loc='upper left')
ax1.set_title('Time vs Position and Reference')
ax1.set_ylabel('Position [deg]')

# 2つ目のグラフ: Current
ax2.plot(filtered_df['time'].to_numpy(), filtered_df['Current_0'].to_numpy(), label='Current_1')
ax2.plot(filtered_df['time'].to_numpy(), filtered_df['Current_1'].to_numpy(), label='Current_2')
ax2.legend(loc='upper left')
ax2.set_title('Time vs Current')
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Current [A]')

# グラフを表示する
plt.tight_layout()
plt.savefig('fig/copy_trac_1_vr.svg', format='svg')

plt.show()