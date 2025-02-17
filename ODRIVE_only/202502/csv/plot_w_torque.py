import pandas as pd
import matplotlib.pyplot as plt


# CSVファイルを読み込む
df = pd.read_csv('/home/naga/klab_ws/ODRIVE_only/202412/csv/two_pos_-0.1_20250217_193825.csv',skiprows=2, usecols=[0,1,3,4,5,7,8], names=['time','Ref_0', 'Position_0', 'Torque_0', 'Ref_1', 'Position_1', 'Torque_1'])

filtered_df = df[(df['time'] >= 0) & (df['time'] <= 20)]

# グラフを作成する
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# 1つ目のグラフ: Position_0, Position_1, Ref_0, Ref_1
ax1.plot(filtered_df['time'].to_numpy(), -filtered_df['Position_0'].to_numpy(), label='Position_0')
ax1.plot(filtered_df['time'].to_numpy(), -filtered_df['Position_1'].to_numpy(), label='Position_1')
ax1.plot(filtered_df['time'].to_numpy(), -filtered_df['Ref_0'].to_numpy(), label='Ref_0', linestyle='--')
ax1.plot(filtered_df['time'].to_numpy(), -filtered_df['Ref_1'].to_numpy(), label='Ref_1', linestyle='--')
# ax1.legend(loc='upper left')
ax1.legend()
ax1.set_ylabel('Position [turns]')
ax1.set_title('Time vs Position and Reference')

# グラフのタイトルと軸ラベルを設定する
plt.title('Time vs Position and Reference')
plt.xlabel('Time [s]')
plt.ylabel('Position [turns]')


# 2つ目のグラフ: Torque
ax2.plot(filtered_df['time'].to_numpy(), filtered_df['Torque_0'].to_numpy(), label='Torque_0', linestyle='-', color='purple')
ax2.plot(filtered_df['time'].to_numpy(), filtered_df['Torque_1'].to_numpy(), label='Torque_1', linestyle='-', color='skyblue')
# ax2.legend(loc='upper left')
ax2.legend()
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Torque [Nm]')
ax2.set_title('Time vs Torque')

# グラフを表示する
plt.tight_layout()

# グラフを表示する
plt.show()