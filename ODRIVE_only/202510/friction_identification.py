"""
friction_identification.py
-------------------------
ODrive等のモータログデータから粘性摩擦係数Bを同定する簡易スクリプト

使い方:
- 実験で得たCSVファイル（トルク・速度ログ）を指定
- 粘性摩擦モデル τ = B * ω でBを推定

必要パッケージ: numpy, pandas, matplotlib, sklearn
"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

# === 設定 ===
CSV_PATH = 'csv/integrated_pid_torque_min_norm_最新.csv'  # 解析したいCSVファイル
TORQUE_COL = 'motor0_torque'  # トルク列名
VEL_COL = 'motor0_vel'        # 速度列名

# === データ読み込み ===
df = pd.read_csv(CSV_PATH)
torque = df[TORQUE_COL].values.reshape(-1, 1)
vel = df[VEL_COL].values.reshape(-1, 1)

# === 0付近のデータ除外（静止摩擦影響を避ける） ===
mask = np.abs(vel) > 0.01
vel_fit = vel[mask]
torque_fit = torque[mask]

# === 線形回帰で粘性摩擦係数Bを推定 ===
reg = LinearRegression(fit_intercept=False)
reg.fit(vel_fit, torque_fit)
B_est = reg.coef_[0][0]
print(f'推定された粘性摩擦係数 B = {B_est:.4f} [Nm/(turn/s)]')

# === グラフ表示 ===
plt.figure(figsize=(6,4))
plt.scatter(vel_fit, torque_fit, s=8, label='Data')
plt.plot(vel_fit, reg.predict(vel_fit), 'r-', label=f'Fit: B={B_est:.3f}')
plt.xlabel('Velocity [turn/s]')
plt.ylabel('Torque [Nm]')
plt.legend()
plt.title('Friction Identification (Viscous)')
plt.grid(True)
plt.tight_layout()
plt.show()
