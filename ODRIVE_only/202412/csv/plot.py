import pandas as pd
import matplotlib.pyplot as plt


# CSVファイルを読み込む
df = pd.read_csv('/home/naga/klab_ws/ODRIVE_only/202412/csv/two_pos_-0.1_20241226_171241.csv',skiprows=2, usecols=[0,1,3,4,6], names=['time','Ref_0', 'Position_0', 'Ref_1', 'Position_1'])

filtered_df = df[(df['time'] >= 0) & (df['time'] <= 4)]

# 横軸をtime、縦軸をPosition_0とPosition_1に設定してプロットする
plt.plot(filtered_df['time'].to_numpy(), -filtered_df['Position_0'].to_numpy(), label='Position_0')
plt.plot(filtered_df['time'].to_numpy(), -filtered_df['Position_1'].to_numpy(), label='Position_1')

## Ref_0とRef_1もプロットする
plt.plot(filtered_df['time'].to_numpy(), -filtered_df['Ref_0'].to_numpy(), label='Ref_0', linestyle='--')
plt.plot(filtered_df['time'].to_numpy(), -filtered_df['Ref_1'].to_numpy(), label='Ref_1', linestyle='--')

# plt.axhline(y=0.4, color='r', linestyle='--')
#plt.axhline(y=-0.4, color='r', linestyle='--')
# 凡例を追加する
plt.legend(loc='upper left')


# グラフのタイトルと軸ラベルを設定する
plt.title('Time vs Position and Reference')
plt.xlabel('Time [s]')
plt.ylabel('Position [rpm]')

# グラフを表示する
plt.show()