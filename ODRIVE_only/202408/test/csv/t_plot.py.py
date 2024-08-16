import pandas as pd
import matplotlib.pyplot as plt

# CSVファイルを読み込む
df = pd.read_csv('sin.csv',skiprows=2, usecols=[1, 0], names=['time', 'time_d'])

# 横軸をtime、縦軸をVelocity_maxonに設定してプロットする
plt.plot(df['time_d'].to_numpy(), df['time'].to_numpy())

# グラフのタイトルと軸ラベルを設定する
plt.title('Time vs Time_d')
plt.xlabel('Time')
plt.ylabel('Time_d')


# グラフを表示する
plt.show()