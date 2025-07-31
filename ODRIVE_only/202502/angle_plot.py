import matplotlib.pyplot as plt
import pandas as pd
import os

# CSVファイルの読み込み
csv_file = '/home/naga/klab_ws/ODRIVE_only/202502/angle/angles_20250303_121138.csv'
df = pd.read_csv(csv_file)

# 重複行を削除
df = df.drop_duplicates()

# フレームを時間（秒）に変換
df['Time (s)'] = df['Frame'] / 30.0

# 角度データを2次元グラフにプロット
plt.figure()
plt.plot(df['Time (s)'], df['Hip-Knee Angle'], label='Hip-Knee Angle')
plt.plot(df['Time (s)'], df['Knee-Ankle Angle'], label='Knee-Ankle Angle')
plt.xlabel('Time (s)')
plt.ylabel('Angle (degrees)')
plt.title('Hip-Knee and Knee-Ankle Angles Over Time')
plt.legend()

# SVGファイルの名前をCSVファイル名に基づいて生成し、svgフォルダ直下に保存
svg_dir = '/home/naga/klab_ws/ODRIVE_only/202502/svg'
os.makedirs(svg_dir, exist_ok=True)
svg_file = os.path.join(svg_dir, os.path.splitext(os.path.basename(csv_file))[0] + '.svg')
plt.savefig(svg_file, format='svg')
plt.show()