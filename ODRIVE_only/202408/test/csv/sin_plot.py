import pandas as pd
import matplotlib.pyplot as plt

# CSVファイルを読み込む
df = pd.read_csv('sin.csv',skiprows=2, usecols=[1, 5], names=['time', 'Position_maxon'])

# 横軸をtime、縦軸をVelocity_maxonに設定してプロットする
plt.plot(df['time'].to_numpy(), df['Position_maxon'].to_numpy())

# グラフのタイトルと軸ラベルを設定する
plt.title('Time vs Velocity')
plt.xlabel('Time')
plt.ylabel('Position')

# グラフを表示する
plt.show()