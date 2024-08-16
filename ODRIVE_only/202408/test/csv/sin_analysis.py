import pandas as pd

# CSVファイルを読み込む
df = pd.read_csv('sin.csv', header=None, skiprows=2)

# 一列目のデータを取得し、その平均を計算する
mean = df[0].mean()

print(mean)