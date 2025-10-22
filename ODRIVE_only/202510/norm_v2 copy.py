"""
トルク制御PID環境 - 最小ノルム配分・ヌル空間安定化 版（コピペ即実行）

目的:
- 出力角θのみを外側PIDで追従
- 出力トルク τ_out* を最小ノルムで各モータへ配分 (τ_cmd = A^T τ_out*)
- ヌル空間ダンピング / 姿勢戻しで内部ドリフトを抑制
- 既存のODriveトルクモードにそのまま繋ぐ

使い方:
1) 本ファイルを保存して `python integrated_pid_torque_control_min_norm.py` を実行。
2) 終了時に CSV とグラフの保存/破棄を選べます。

注意:
- ODriveのシリアル番号やトルク定数は環境に合わせて変更してください。
- 出力角θは独立エンコーダ (odrv2) で計測する想定です。
"""

# ===================== 標準ライブラリ =====================
import csv
import json
import math
import os
import threading
import time
from datetime import datetime

# ===================== サードパーティ =====================
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ===================== ODrive =====================
import odrive
from odrive.enums import *

# ===================== 日本語フォント設定 =====================
plt.rcParams['font.family'] = 'DejaVu Sans'
japanese_fonts = ['Noto Sans CJK JP', 'Hiragino Sans', 'Yu Gothic', 'Meiryo', 'Takao', 'IPAexGothic', 'IPAPGothic', 'VL PGothic']
for font in japanese_fonts:
    if font in [f.name for f in fm.fontManager.ttflist]:
        plt.rcParams['font.family'] = font
        break
else:
    print("警告: 日本語フォントが見つかりません。グラフは英語表記になります。")

# ==================================================================================
# 設定
# ==================================================================================
STEP_CONFIG = {
    'initial_wait': 1.0,        # 最初のステップまでの待機[秒]
    'step_duration': 3.0,       # 各ステップの持続[秒]
    'output_amplitude': 0.3,    # 出力θのステップ振幅[turn]
}

# 出力θのPIDゲイン（外側）
OUTPUT_PID = {'kp': 1.0, 'ki': 0.0, 'kd': 0.1, 'max_output': 1.0}

# ヌル空間安定化ゲイン
NULLSPACE_CONFIG = {
    'Knu_diag': [0.3, 0.3],   # 粘性ダンピング（対角）
    'Kq_diag':  [0.0, 0.0],   # 姿勢戻し（まずは0）
    'q_ref':    [0.0, 0.0],   # 望ましい関節姿勢
}

# 安全制限
SAFETY_CONFIG = {
    'max_torque0': 1.0,      # T-motor 最大トルク[Nm]
    'max_torque1': 0.1,      # Maxon 最大トルク[Nm]
}

# ODrive接続設定（必要に応じて変更）
ODRIVE_SERIAL = {
    'motor0': '3856345D3539',  # T-motor 側
    'motor1': '384D346F3539',  # Maxon 側
    'output': '3849346F3539',  # 出力エンコーダ
}
ODRIVE_TORQUE_CONSTANT = {
    'motor0': 0.106,  # Nm/A
    'motor1': 0.091,  # Nm/A
}

# 制御周期 [Hz]
CONTROL_FREQUENCY = 200

# ==================================================================================
# ユーティリティ: 機構行列 / 最小ノルム配分 / ヌル空間射影
# ==================================================================================

def get_A(q=None):
    """機構の出力写像 A = [a1 a2] (1x2)
    姿勢依存の場合は q から計算する。まずは定数でOK。
    例: a1 = 1/20, a2 = 163/2000
    """
    return np.array([[1/20, 163/2000]])


def project_null(A):
    """ヌル空間射影 P_N = I - A^T (A A^T)^(-1) A (2x2)
    Aは1x2、AA^Tは1x1スカラーなので安定化のためにダンピングを入れる。
    """
    At = A.T
    s = float(A @ At)   # = a1^2 + a2^2
    eps = 1e-6
    inv = 1.0 / max(s, eps)
    PN = np.eye(2) - At * inv @ A
    return PN


def min_norm_torque_split(A, tau_out):
    """最小ノルムのトルク配分: τ* = A^T (A A^T)^(-1) τ_out"""
    At = A.T
    s = float(A @ At)  # = a1^2 + a2^2
    if s < 1e-8:
        raise ValueError("Mechanism matrix A is near-singular.")
    scale = float(tau_out) / s
    return (At * scale).reshape(2)

# ==================================================================================
# 目標生成
# ==================================================================================

def generate_output_step(elapsed_time):
    """出力(θ)のステップ目標値を生成"""
    initial_wait = STEP_CONFIG['initial_wait']
    step_duration = STEP_CONFIG['step_duration']
    amp = STEP_CONFIG['output_amplitude']

    if elapsed_time < initial_wait:
        return 0.0

    cyc = (elapsed_time - initial_wait) % (step_duration * 4)
    if cyc < step_duration:
        return +amp
    elif cyc < step_duration * 2:
        return 0.0
    elif cyc < step_duration * 3:
        return -amp
    else:
        return 0.0

# ==================================================================================
# PID コントローラ（スレッドセーフ）
# ==================================================================================

class PIDController:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, max_output=10.0, min_output=-10.0):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.max_output = max_output; self.min_output = min_output
        self.prev_error = 0.0; self.integral = 0.0; self.prev_time = time.time()
        self.lock = threading.Lock()

    def update(self, setpoint, feedback):
        with self.lock:
            t = time.time(); dt = t - self.prev_time
            if dt <= 0.0: dt = 1e-6
            e = setpoint - feedback
            P = self.kp * e
            self.integral += e * dt
            if self.ki > 0:
                i_lim = self.max_output / self.ki
                self.integral = max(min(self.integral, i_lim), -i_lim)
            I = self.ki * self.integral
            D = self.kd * (e - self.prev_error) / dt
            u = P + I + D
            u = max(min(u, self.max_output), self.min_output)
            self.prev_error = e; self.prev_time = t
            return u, e, P, I, D

# ==================================================================================
# 可視化/解析
# ==================================================================================

def analyze_and_plot_step_response(csv_filename):
    print(f"ステップ応答解析を開始: {csv_filename}")
    df = pd.read_csv(csv_filename)


    fig, axes = plt.subplots(4, 1, figsize=(12, 12))
    fig.suptitle('Output Step Response (θ) and Torques')

    t = df['time'].values
    theta_ref = df['theta_ref'].values
    theta = df['output_pos'].values
    tau_out = df['tau_out_ref'].values
    tau0 = df['motor0_torque'].values
    tau1 = df['motor1_torque'].values
    theta1 = df['motor0_pos'].values
    theta2 = df['motor1_pos'].values

    # θ
    axes[0].plot(t, theta_ref, '--', label='θ_ref')
    axes[0].plot(t, theta, '-', label='θ')
    axes[0].set_ylabel('θ [turn]')
    axes[0].legend()

    # τ_out
    axes[1].plot(t, tau_out, '-')
    axes[1].set_ylabel('τ_out [Nm]')

    # τ0, τ1
    axes[2].plot(t, tau0, color='red', label='τ1')
    axes[2].plot(t, tau1, color='green', label='τ2')
    axes[2].set_ylabel('Torque [Nm]')
    axes[2].legend()

    # 内部姿勢 θ1, θ2
    axes[3].plot(t, theta1, color='red', label='θ1')
    axes[3].plot(t, theta2, color='green', label='θ2')
    axes[3].set_xlabel('Time [s]')
    axes[3].set_ylabel('Joint [turn]')
    axes[3].legend()

    # グリッド線なし、目盛り内向き
    for ax in axes:
        ax.grid(False)
        ax.tick_params(axis='both', direction='in', length=6, width=0.8)

    plt.tight_layout()

    os.makedirs('fig', exist_ok=True)
    base = os.path.basename(csv_filename).replace('.csv', '_step_response.pdf')
    fig_path = os.path.join('fig', base)
    plt.savefig(fig_path, dpi=300, bbox_inches='tight', format='pdf')
    plt.show()

    print("\n" + "="*60)
    print("データファイルの処理を選択してください:")
    print("  [1] CSVとグラフの両方を保存")
    print("  [2] グラフのみ保存（CSVは破棄）")
    print("  [3] CSVのみ保存（グラフは破棄）")
    print("  [4] 両方とも破棄")
    print("="*60)

    while True:
        try:
            choice = input("選択 (1-4): ").strip()
            if choice in ['1','2','3','4']:
                break
            else:
                print("1,2,3,4 のいずれかを入力してください。")
        except KeyboardInterrupt:
            print("\n処理をキャンセルします。")
            choice = '4'
            break

    final_csv_path = csv_filename
    final_graph_path = fig_path

    try:
        if choice == '1':
            print(f"✅ CSV保存: {csv_filename}")
            print(f"✅ グラフ保存: {fig_path}")
        elif choice == '2':
            os.remove(csv_filename); final_csv_path = None
            print(f"🗑️ CSV削除: {csv_filename}")
            print(f"✅ グラフ保存: {fig_path}")
        elif choice == '3':
            os.remove(fig_path); final_graph_path = None
            print(f"✅ CSV保存: {csv_filename}")
            print(f"🗑️ グラフ削除: {fig_path}")
        elif choice == '4':
            os.remove(csv_filename); os.remove(fig_path)
            final_csv_path = None; final_graph_path = None
            print(f"🗑️ 両方削除")
    except Exception as e:
        print(f"⚠️ ファイル削除エラー: {e}")

    plt.close('all')
    return final_graph_path, final_csv_path

# ==================================================================================
# メイン
# ==================================================================================

def main():
    print("=== トルク制御PID環境 - 最小ノルム配分 版 ===")

    # ---- ODrive 接続 ----
    print("ODriveを検索中...")
    try:
        odrv0 = odrive.find_any(serial_number=ODRIVE_SERIAL['motor0'])
        odrv1 = odrive.find_any(serial_number=ODRIVE_SERIAL['motor1'])
        odrv2 = odrive.find_any(serial_number=ODRIVE_SERIAL['output'])
        print("ODrive接続完了")
    except Exception as e:
        print(f"ODrive接続エラー: {e}")
        return

    # 初期位置
    initial_position0 = odrv0.axis0.pos_vel_mapper.pos_rel
    initial_position1 = odrv1.axis0.pos_vel_mapper.pos_rel
    initial_position2 = odrv2.axis0.pos_vel_mapper.pos_rel

    # ---- モータをトルク制御モードへ ----
    print("モータをトルク制御モードに設定中...")
    odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
    odrv0.axis0.config.motor.torque_constant = ODRIVE_TORQUE_CONSTANT['motor0']

    odrv1.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    odrv1.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
    odrv1.axis0.config.motor.torque_constant = ODRIVE_TORQUE_CONSTANT['motor1']
    print("モータ設定完了")

    # ---- コントローラ ----
    output_pid = PIDController(kp=OUTPUT_PID['kp'], ki=OUTPUT_PID['ki'], kd=OUTPUT_PID['kd'], max_output=OUTPUT_PID['max_output'])

    # ---- ログ ----
    data_lock = threading.Lock()
    data_log = {
        'time': [],
        'motor0': {'pos': [], 'vel': [], 'torque': []},
        'motor1': {'pos': [], 'vel': [], 'torque': []},
        'output': {'pos': [], 'vel': []},
        'theta_ref': [],
        'tau_out_ref': [],
    }

    # ---- ヌル空間ゲイン ----
    Knu = np.diag(NULLSPACE_CONFIG['Knu_diag'])
    Kq  = np.diag(NULLSPACE_CONFIG['Kq_diag'])
    q_ref = np.array(NULLSPACE_CONFIG['q_ref'])

    start_time = time.time()
    dt_target = 1.0 / CONTROL_FREQUENCY

    print("=== 制御開始 (Ctrl+Cで停止) ===")

    try:
        while True:
            t0 = time.time()
            elapsed = t0 - start_time

            # ---- 目標 ----
            theta_ref = generate_output_step(elapsed)

            # ---- 計測 ----
            q0 = odrv0.axis0.pos_vel_mapper.pos_rel - initial_position0
            q1 = odrv1.axis0.pos_vel_mapper.pos_rel - initial_position1
            qout = odrv2.axis0.pos_vel_mapper.pos_rel - initial_position2
            dq0 = odrv0.axis0.pos_vel_mapper.vel
            dq1 = odrv1.axis0.pos_vel_mapper.vel
            dqout = odrv2.axis0.pos_vel_mapper.vel  # 参考

            q = np.array([q0, q1])
            qdot = np.array([dq0, dq1])

            # ---- 機構行列・射影 ----
            A = get_A(q)
            PN = project_null(A)

            # ---- 出力PID → 望ましい出力トルク ----
            tau_out_ref, _, _, _, _ = output_pid.update(theta_ref, qout)

            # ---- 最小ノルム配分 + ヌル空間安定化 ----
            tau_min = min_norm_torque_split(A, tau_out_ref)
            tau_null = (-Knu @ (PN @ qdot) - Kq @ (PN @ (q - q_ref)))
            tau_cmd = tau_min + tau_null

            # ---- 飽和 ----
            tau_cmd[0] = float(np.clip(tau_cmd[0], -SAFETY_CONFIG['max_torque0'], SAFETY_CONFIG['max_torque0']))
            tau_cmd[1] = float(np.clip(tau_cmd[1], -SAFETY_CONFIG['max_torque1'], SAFETY_CONFIG['max_torque1']))

            # ---- 出力 ----
            odrv0.axis0.controller.input_torque = tau_cmd[0]
            odrv1.axis0.controller.input_torque = tau_cmd[1]

            # ---- ログ ----
            with data_lock:
                data_log['time'].append(elapsed)
                data_log['motor0']['pos'].append(q0)
                data_log['motor0']['vel'].append(dq0)
                data_log['motor0']['torque'].append(float(tau_cmd[0]))
                data_log['motor1']['pos'].append(q1)
                data_log['motor1']['vel'].append(dq1)
                data_log['motor1']['torque'].append(float(tau_cmd[1]))
                data_log['output']['pos'].append(qout)
                data_log['output']['vel'].append(dqout)
                data_log['theta_ref'].append(theta_ref)
                data_log['tau_out_ref'].append(float(tau_out_ref))

            # ---- タイミング調整 ----
            dt = time.time() - t0
            sleep = dt_target - dt
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print("\n制御を停止中...")
    finally:
        try:
            odrv0.axis0.controller.input_torque = 0.0
            odrv1.axis0.controller.input_torque = 0.0
            odrv0.axis0.requested_state = AxisState.IDLE
            odrv1.axis0.requested_state = AxisState.IDLE
        except Exception:
            pass

        # ---- CSV保存 ----
        os.makedirs('csv', exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = f'csv/integrated_pid_torque_min_norm_{timestamp}.csv'
        with open(csv_filename, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                'time',
                'motor0_pos','motor0_vel','motor0_torque',
                'motor1_pos','motor1_vel','motor1_torque',
                'output_pos','output_vel',
                'theta_ref','tau_out_ref'
            ])
            for i in range(len(data_log['time'])):
                w.writerow([
                    data_log['time'][i],
                    data_log['motor0']['pos'][i], data_log['motor0']['vel'][i], data_log['motor0']['torque'][i],
                    data_log['motor1']['pos'][i], data_log['motor1']['vel'][i], data_log['motor1']['torque'][i],
                    data_log['output']['pos'][i], data_log['output']['vel'][i],
                    data_log['theta_ref'][i], data_log['tau_out_ref'][i]
                ])
        print(f"データ保存完了: {csv_filename}")

        # ---- 可視化 ----
        try:
            print("\n=== ステップ応答解析とグラフ表示 ===")
            final_graph_path, final_csv_path = analyze_and_plot_step_response(csv_filename)
            print("\n=== 最終的なファイル状況 ===")
            print(f"CSV: {final_csv_path if final_csv_path else '削除済み'}")
            print(f"FIG: {final_graph_path if final_graph_path else '削除済み'}")
        except Exception as e:
            print(f"ステップ応答解析エラー: {e}")
            print("手動でCSVファイルを確認してください。")
        print("制御終了")


if __name__ == '__main__':
    main()
