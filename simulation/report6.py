import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# システムのパラメータ
Ke = 100  # 環境剛性 [N/m]
De = 10   # 粘性係数 [Ns/m]
Mv = 1    # 仮想質量

# ダイナミクスモデル
def force_control_system(state, t, F_cmd, Ke, De, Mv):
    theta, theta_dot = state
    # 制御方程式
    F_res = Ke * (theta) + De * theta_dot
    theta_ddot = (F_cmd - F_res) / Mv
    return [theta_dot, theta_ddot]

# シミュレーション設定
time = np.linspace(0, 5, 1000)  # 時間
initial_state = [0, 0]  # 初期条件 [θ, dθ/dt]
F_cmd = 10  # 指令値

# シミュレーション実行
result = odeint(force_control_system, initial_state, time, args=(F_cmd, Ke, De, Mv))

# 結果プロット
theta = result[:, 0]
plt.plot(time, theta)
plt.title("Force Response")
plt.xlabel("Time [s]")
plt.ylabel("Theta [rad]")
plt.grid()
plt.show()
