import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# システムのパラメータ
Ke = 100  # 環境剛性 [N/m]
De = 10   # 粘性係数 [Ns/m]
Mv = 1    # 仮想質量
D_gain = 50  # D制御のゲイン

# システムダイナミクス（制御なし）
def open_loop(state, t, F_cmd, Ke, De, Mv):
    theta, theta_dot = state  # 状態 [角度, 角速度]
    
    # 応答力 (システムの出力)
    F_res = Ke * theta + De * theta_dot
    
    # 入力力
    F_input = F_cmd
    
    # 状態方程式 (角加速度)
    theta_ddot = (F_input - F_res) / Mv
    
    return [theta_dot, theta_ddot]

# システムダイナミクス（D制御）
def force_control_d(state, t, F_cmd, Ke, De, Mv, D_gain, dt):
    theta, theta_dot, prev_error = state  # 状態 [角度, 角速度, 前のエラー]
    
    # 応答力 (システムの出力)
    F_res = Ke * theta + De * theta_dot
    
    # エラーとその微分
    error = F_cmd - F_res
    d_error = (error - prev_error) / dt  # 時間間隔で微分近似
    
    # D制御による補正力
    F_control = D_gain * d_error
    
    # 合計入力力
    F_input = F_cmd + F_control
    
    # 状態方程式 (角加速度)
    theta_ddot = (F_input - F_res) / Mv
    
    return [theta_dot, theta_ddot, error]

# 時間設定
time = np.linspace(0, 5, 1000)  # シミュレーション時間 [0, 5秒]
dt = time[1] - time[0]

# 初期条件
initial_state_open = [0, 0]       # 制御なし: [角度, 角速度]
initial_state_d = [0, 0, 0]       # D制御: [角度, 角速度, 前のエラー]

# 指令力
F_cmd = 10  # 定数指令力

# シミュレーション実行
result_open = odeint(open_loop, initial_state_open, time, args=(F_cmd, Ke, De, Mv))
result_d = odeint(force_control_d, initial_state_d, time, args=(F_cmd, Ke, De, Mv, D_gain, dt))

# 結果の取得
theta_open = result_open[:, 0]
theta_d = result_d[:, 0]

# グラフのプロット
plt.figure(figsize=(10, 6))

# グラフ1: 制御なし
plt.subplot(2, 1, 1)
plt.plot(time, theta_open, label="Open-Loop Response (No Control)", color="blue")
plt.title("Open-Loop vs D-Control Response")
plt.xlabel("Time [s]")
plt.ylabel("Theta [rad]")
plt.grid()
plt.legend()

# グラフ2: D制御
plt.subplot(2, 1, 2)
plt.plot(time, theta_d, label="D-Control Response", color="green")
plt.xlabel("Time [s]")
plt.ylabel("Theta [rad]")
plt.grid()
plt.legend()

plt.tight_layout()
plt.show()
