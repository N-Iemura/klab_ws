"""
トルク制御PID環境 - 統合版
角度と速度の指令値に対するPIDゲイン調整が可能な環境

使用方法:
    # モータ別のステップ振幅[turn]
    'motor0_amplitude': 0.1,    # T-motorのステップ高さ
    'motor1_amplitude': 0.05,   # Maxonのステップ高さ（振動対策で小さく）python integrated_pid_torque_control.py を実行
2. 別ターミナルで python pid_gain_adjuster_integrated.py を実行してリアルタイム調整

特徴:
- 各モータの位置・速度に対する独立PID制御
- トルク制御による精密制御
- リアルタイムゲイン調整対応
- 詳細なデータログ記録
"""

# 標準ライブラリ
import csv
import json
import math
import os
import threading
import time
from datetime import datetime

# サードパーティライブラリ
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ODriveライブラリ
import odrive
from odrive.enums import *

# 日本語フォント設定
plt.rcParams['font.family'] = 'DejaVu Sans'  # デフォルトフォント
# 利用可能な日本語フォントを設定（システムにより異なる）
japanese_fonts = ['Noto Sans CJK JP', 'Hiragino Sans', 'Yu Gothic', 'Meiryo', 'Takao', 'IPAexGothic', 'IPAPGothic', 'VL PGothic']
for font in japanese_fonts:
    if font in [f.name for f in fm.fontManager.ttflist]:
        plt.rcParams['font.family'] = font
        break
else:
    # 日本語フォントが見つからない場合は、文字化けを避けるため英語表記に変更
    print("警告: 日本語フォントが見つかりません。グラフは英語表記になります。")

# ==================================================================================
# モータ制御設定 - ここを編集してステップ応答パターンをカスタマイズ
# ==================================================================================

# ステップ応答パターン設定
STEP_CONFIG = {
    # タイミング設定
    'initial_wait': 1.0,        # 最初のステップまでの待機時間[秒]
    'step_duration': 3.0,       # 各ステップの持続時間[秒]
    
    # モータ別のステップ振幅[turn]
    'motor0_amplitude': 0.3,    # T-motorのステップ高さ
    'motor1_amplitude': -0.3,    # Maxonのステップ高さ
    
    # ステップパターン（True=有効, False=無効）
    'motor0_enabled': True,     # Motor0を動かすか
    'motor1_enabled': True,     # Motor1を動かすか
    
    # ステップの種類（'both'=同期, 'alternate'=交互, 'opposite'=逆位相）
    'pattern_type': 'both'      # 'both', 'alternate', 'opposite'
}

# PIDゲイン設定
PID_CONFIG = {
    'motor0': {'kp': 6.8, 'ki': 0.0, 'kd': 0.25, 'max_output': 5.0},   # T-motor
    'motor1': {'kp': 0.9, 'ki': 0.0, 'kd': 0.0125, 'max_output': 0.1}     # Maxon (振動対策で大幅に低減)
}

# 安全制限設定
SAFETY_CONFIG = {
    'max_torque0': 6.0,         # T-motorの最大トルク[Nm]
    'max_torque1': 0.1,         # Maxonの最大トルク[Nm]（振動対策で低減）
}

# ==================================================================================

# PID制御クラス
class PIDController:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, max_output=10.0, min_output=-10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()
        self.lock = threading.Lock()  # スレッドセーフ
        
    def update(self, setpoint, feedback):
        with self.lock:
            current_time = time.time()
            dt = current_time - self.prev_time
            
            if dt <= 0.0:
                dt = 1e-6
                
            error = setpoint - feedback
            
            # 比例項
            proportional = self.kp * error
            
            # 積分項（ワインドアップ対策）
            self.integral += error * dt
            if self.ki > 0:
                integral_limit = self.max_output / self.ki
                self.integral = max(min(self.integral, integral_limit), -integral_limit)
            integral_term = self.ki * self.integral
            
            # 微分項
            derivative = (error - self.prev_error) / dt
            derivative_term = self.kd * derivative
            
            # PID出力
            output = proportional + integral_term + derivative_term
            
            # 出力制限
            output = max(min(output, self.max_output), self.min_output)
            
            # 次回のために保存
            self.prev_error = error
            self.prev_time = current_time
            
            return output, error, proportional, integral_term, derivative_term
    
    def reset(self):
        with self.lock:
            self.prev_error = 0.0
            self.integral = 0.0
            self.prev_time = time.time()
            
    def update_gains(self, kp=None, ki=None, kd=None):
        with self.lock:
            if kp is not None:
                self.kp = kp
            if ki is not None:
                self.ki = ki
            if kd is not None:
                self.kd = kd

# グローバル変数（ゲイン調整用）
pid_controllers = {}
control_running = False
data_lock = threading.Lock()

def save_gains_to_file():
    """現在のゲインをJSONファイルに保存"""
    gains_data = {}
    for name, controller in pid_controllers.items():
        gains_data[name] = {
            'kp': controller.kp,
            'ki': controller.ki,
            'kd': controller.kd
        }
    
    filename = f"gain/current_gains_{int(time.time())}.json"
    with open(filename, 'w') as f:
        json.dump(gains_data, f, indent=2)
    return filename

def load_gains_from_file(filename):
    """JSONファイルからゲインを読み込み"""
    try:
        with open(filename, 'r') as f:
            gains_data = json.load(f)
        
        for name, gains in gains_data.items():
            if name in pid_controllers:
                pid_controllers[name].update_gains(
                    kp=gains['kp'],
                    ki=gains['ki'],
                    kd=gains['kd']
                )
        return True
    except Exception as e:
        print(f"ゲイン読み込みエラー: {e}")
        return False

def generate_step_targets(elapsed_time):
    """設定に基づいてステップ目標値を生成"""
    initial_wait = STEP_CONFIG['initial_wait']
    step_duration = STEP_CONFIG['step_duration']
    
    # 初期待機時間
    if elapsed_time < initial_wait:
        return 0.0, 0.0
    
    # ステップサイクル計算
    step_cycle = (elapsed_time - initial_wait) % (step_duration * 4)
    
    # 基本ステップパターン（-1, 0, +1, 0）
    if step_cycle < step_duration:
        # 第1段階: +ステップ
        step_value = 1.0
    elif step_cycle < step_duration * 2:
        # 第2段階: 0
        step_value = 0.0
    elif step_cycle < step_duration * 3:
        # 第3段階: -ステップ
        step_value = -1.0
    else:
        # 第4段階: 0
        step_value = 0.0
    
    # モータ別の目標値計算
    target_pos0 = 0.0
    target_pos1 = 0.0
    
    if STEP_CONFIG['motor0_enabled']:
        target_pos0 = step_value * STEP_CONFIG['motor0_amplitude']
    
    if STEP_CONFIG['motor1_enabled']:
        if STEP_CONFIG['pattern_type'] == 'both':
            # 同期パターン
            target_pos1 = step_value * STEP_CONFIG['motor1_amplitude']
        elif STEP_CONFIG['pattern_type'] == 'opposite':
            # 逆位相パターン
            target_pos1 = -step_value * STEP_CONFIG['motor1_amplitude']
        elif STEP_CONFIG['pattern_type'] == 'alternate':
            # 交互パターン（Motor0が動いている時はMotor1は止まる）
            if abs(step_value) > 0.5:
                target_pos1 = 0.0
            else:
                # Motor0が0の時にMotor1が動く
                alt_cycle = ((elapsed_time - initial_wait) / step_duration) % 4
                if 1 <= alt_cycle < 2 or 3 <= alt_cycle < 4:
                    target_pos1 = STEP_CONFIG['motor1_amplitude'] if alt_cycle < 2 else -STEP_CONFIG['motor1_amplitude']
    
    return target_pos0, target_pos1

def analyze_and_plot_step_response(csv_filename):
    """CSVデータからステップ応答を解析してグラフを表示"""
    print(f"ステップ応答解析を開始: {csv_filename}")
    
    # CSVファイル読み込み
    df = pd.read_csv(csv_filename)
    
    # グラフ作成
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle('Step Response Analysis Results', fontsize=16)
    
    # Motor 0 の解析
    analyze_motor_response(df, 0, axes[:, 0])
    
    # Motor 1 の解析
    analyze_motor_response(df, 1, axes[:, 1])
    
    plt.tight_layout()
    
    # 一時的にグラフファイルを作成（表示用）
    os.makedirs('fig', exist_ok=True)
    base_filename = os.path.basename(csv_filename).replace('.csv', '_step_response.pdf')
    temp_graph_filename = os.path.join('fig', base_filename)
    plt.savefig(temp_graph_filename, dpi=300, bbox_inches='tight', format='pdf')
    
    # グラフを表示
    plt.show()
    
    # ユーザーに保存/破棄の選択を求める
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
            if choice in ['1', '2', '3', '4']:
                break
            else:
                print("1, 2, 3, 4 のいずれかを入力してください。")
        except KeyboardInterrupt:
            print("\n処理をキャンセルします。")
            choice = '4'
            break
    
    # 選択に応じてファイル処理
    final_csv_path = csv_filename
    final_graph_path = temp_graph_filename
    
    if choice == '1':
        # 両方保存
        print(f"✅ CSVファイルを保存: {csv_filename}")
        print(f"✅ グラフファイルを保存: {temp_graph_filename}")
    elif choice == '2':
        # グラフのみ保存、CSVは破棄
        try:
            os.remove(csv_filename)
            print(f"🗑️  CSVファイルを削除: {csv_filename}")
            print(f"✅ グラフファイルを保存: {temp_graph_filename}")
            final_csv_path = None
        except Exception as e:
            print(f"⚠️  CSV削除エラー: {e}")
    elif choice == '3':
        # CSVのみ保存、グラフは破棄
        try:
            os.remove(temp_graph_filename)
            print(f"✅ CSVファイルを保存: {csv_filename}")
            print(f"🗑️  グラフファイルを削除: {temp_graph_filename}")
            final_graph_path = None
        except Exception as e:
            print(f"⚠️  グラフ削除エラー: {e}")
    elif choice == '4':
        # 両方破棄
        try:
            os.remove(csv_filename)
            os.remove(temp_graph_filename)
            print(f"🗑️  CSVファイルを削除: {csv_filename}")
            print(f"🗑️  グラフファイルを削除: {temp_graph_filename}")
            final_csv_path = None
            final_graph_path = None
        except Exception as e:
            print(f"⚠️  ファイル削除エラー: {e}")
    
    plt.close('all')  # メモリリークを防ぐためにプロットを閉じる
    
    return final_graph_path, final_csv_path

def analyze_motor_response(df, motor_id, axes):
    """個別モータのステップ応答解析"""
    target_col = f'motor{motor_id}_setpoint_pos'
    actual_col = f'motor{motor_id}_pos'
    error_col = f'motor{motor_id}_error_pos'
    torque_col = f'motor{motor_id}_torque'
    
    time_data = df['time'].values
    target_data = df[target_col].values
    actual_data = df[actual_col].values
    error_data = df[error_col].values
    torque_data = df[torque_col].values
    
    # 1. Position Step Response
    axes[0].plot(time_data, target_data, 'r--', label='Target', linewidth=2)
    axes[0].plot(time_data, actual_data, 'b-', label='Actual', linewidth=1)
    axes[0].set_title(f'Motor {motor_id} - Position Step Response')
    axes[0].set_ylabel('Position [turn]')
    axes[0].grid(True)
    axes[0].legend()
    
    # 2. Torque Output
    axes[1].plot(time_data, torque_data, 'g-', linewidth=1)
    axes[1].set_title(f'Motor {motor_id} - Torque Output')
    axes[1].set_ylabel('Torque [Nm]')
    axes[1].grid(True)
    
    # 3. Position Error
    axes[2].plot(time_data, error_data, 'r-', linewidth=1)
    axes[2].set_title(f'Motor {motor_id} - Position Error')
    axes[2].set_ylabel('Error [turn]')
    axes[2].set_xlabel('Time [s]')
    axes[2].grid(True)
    
    # ステップ応答特性の数値解析
    print(f"\n=== Motor {motor_id} ステップ応答特性 ===")
    
    # ステップ変化点を検出
    target_diff = np.diff(target_data)
    step_indices = np.where(np.abs(target_diff) > 0.05)[0]
    
    if len(step_indices) > 0:
        for i, step_idx in enumerate(step_indices[:2]):  # 最初の2つのステップを解析
            step_start = step_idx + 1
            step_end = min(step_start + 1000, len(df))  # 応答解析範囲
            
            target_val = target_data[step_start]
            if abs(target_val) < 0.01:  # 0への変化はスキップ
                continue
                
            actual_vals = actual_data[step_start:step_end]
            time_vals = time_data[step_start:step_end] - time_data[step_start]
            
            print(f"\nステップ {i+1}: 目標値 {target_val:.3f} turn")
            
            # 立ち上がり時間（10%-90%）
            final_val = target_val
            val_10 = final_val * 0.1
            val_90 = final_val * 0.9
            
            if target_val > 0:
                rise_start_idx = np.where(actual_vals >= val_10)[0]
                rise_end_idx = np.where(actual_vals >= val_90)[0]
            else:
                rise_start_idx = np.where(actual_vals <= val_10)[0]
                rise_end_idx = np.where(actual_vals <= val_90)[0]
            
            if len(rise_start_idx) > 0 and len(rise_end_idx) > 0:
                rise_time = time_vals[rise_end_idx[0]] - time_vals[rise_start_idx[0]]
                print(f"  立ち上がり時間: {rise_time:.3f}秒")
            
            # オーバーシュート
            if target_val > 0:
                max_val = np.max(actual_vals)
                overshoot = ((max_val - final_val) / final_val) * 100 if final_val != 0 else 0
            else:
                max_val = np.min(actual_vals)
                overshoot = ((final_val - max_val) / abs(final_val)) * 100 if final_val != 0 else 0
            print(f"  オーバーシュート: {overshoot:.1f}%")
            
            # 整定時間（5%以内）
            settle_threshold = 0.05 * abs(final_val)
            settle_indices = np.where(np.abs(actual_vals - final_val) <= settle_threshold)[0]
            if len(settle_indices) > 50:  # 連続して条件を満たす
                settle_time = time_vals[settle_indices[50]]
                print(f"  整定時間(5%): {settle_time:.3f}秒")
            
            # 定常偏差
            steady_error = np.mean(error_data[step_end-100:step_end])
            print(f"  定常偏差: {steady_error:.4f} turn")

def print_current_config():
    """現在の設定を表示"""
    print("\n=== 現在の制御設定 ===")
    print(f"ステップパターン: {STEP_CONFIG['pattern_type']}")
    print(f"Motor0 振幅: {STEP_CONFIG['motor0_amplitude']} turn ({'有効' if STEP_CONFIG['motor0_enabled'] else '無効'})")
    print(f"Motor1 振幅: {STEP_CONFIG['motor1_amplitude']} turn ({'有効' if STEP_CONFIG['motor1_enabled'] else '無効'})")
    print(f"ステップ持続時間: {STEP_CONFIG['step_duration']} 秒")
    print(f"初期待機時間: {STEP_CONFIG['initial_wait']} 秒")
    
    print("\n=== PIDゲイン設定 ===")
    for motor, config in PID_CONFIG.items():
        print(f"{motor}: kp={config['kp']}, ki={config['ki']}, kd={config['kd']}, max_output={config['max_output']}")
    
    print("\n=== 安全制限設定 ===")
    print(f"Motor0 最大トルク: {SAFETY_CONFIG['max_torque0']} Nm")
    print(f"Motor1 最大トルク: {SAFETY_CONFIG['max_torque1']} Nm")
    print("=" * 50)

def calc_min_norm_torque(tau_out_ref):
    """
    出力軸トルクtau_out_refを、最小ノルムで2モータに分配する
    tau = A⁺ * tau_out_ref
    """
    # 機構伝達行列
    A = np.array([[1/20, 163/2000]])  # shape (1,2)
    # ムーア・ペンローズ擬似逆行列
    A_pinv = np.linalg.pinv(A)        # shape (2,1)
    tau = np.dot(A_pinv, tau_out_ref) # shape (2,)
    return tau[0], tau[1]

def main():
    global pid_controllers, control_running
    
    print("=== トルク制御PID環境 - 統合版 ===")
    
    # 現在の設定を表示
    print_current_config()
    
    # ODrive接続
    print("ODriveを検索中...")
    try:
        odrv0 = odrive.find_any(serial_number='3856345D3539')  # T-motor
        odrv1 = odrive.find_any(serial_number='384D346F3539')  # Maxon
        odrv2 = odrive.find_any(serial_number='3849346F3539')  # Encoder
        print("ODrive接続完了")
    except Exception as e:
        print(f"ODrive接続エラー: {e}")
        return
    
    # 初期位置の記録
    initial_position0 = odrv0.axis0.pos_vel_mapper.pos_rel
    initial_position1 = odrv1.axis0.pos_vel_mapper.pos_rel
    initial_position2 = odrv2.axis0.pos_vel_mapper.pos_rel
    
    # モータ設定（トルク制御モード）
    print("モータをトルク制御モードに設定中...")
    odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
    odrv0.axis0.config.motor.torque_constant = 0.106
    
    odrv1.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    odrv1.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
    odrv1.axis0.config.motor.torque_constant = 0.091
    print("モータ設定完了")
    
    # PIDコントローラ初期化
    print("PIDコントローラを初期化中...")
    
    # PIDゲイン設定を読み込み
    pid_controllers = {
        'Motor0_Position': PIDController(
            kp=PID_CONFIG['motor0']['kp'],
            ki=PID_CONFIG['motor0']['ki'], 
            kd=PID_CONFIG['motor0']['kd'],
            max_output=PID_CONFIG['motor0']['max_output']
        ),
        'Motor1_Position': PIDController(
            kp=PID_CONFIG['motor1']['kp'],
            ki=PID_CONFIG['motor1']['ki'],
            kd=PID_CONFIG['motor1']['kd'], 
            max_output=PID_CONFIG['motor1']['max_output']
        )
    }
    print("PIDコントローラ初期化完了")
    
    # データ記録用
    data_log = {
        'time': [],
        'motor0': {'pos': [], 'vel': [], 'torque': [], 'setpoint_pos': [], 'setpoint_vel': [], 'error_pos': [], 'error_vel': []},
        'motor1': {'pos': [], 'vel': [], 'torque': [], 'setpoint_pos': [], 'setpoint_vel': [], 'error_pos': [], 'error_vel': []},
        'output': {'pos': [], 'vel': []}
    }
    
    # 制御パラメータ
    start_time = time.time()
    control_frequency = 200  # Hz
    dt_target = 1.0 / control_frequency
    
    # 目標値生成パラメータ
    position_amplitude = 0.3
    velocity_amplitude = 1.5
    frequency = 0.15
    
    print("=== 制御開始 ===")
    print("リアルタイムゲイン調整: python pid_gain_adjuster_integrated.py")
    print("Ctrl+Cで終了")
    
    control_running = True
    loop_count = 0
    
    # 出力軸PIDコントローラ（ループ外で初期化）
    output_pid = PIDController(kp=6.0, ki=0.0, kd=0.2, max_output=6.0)

    try:
        while control_running:
            loop_start_time = time.time()
            elapsed_time = loop_start_time - start_time

            # ステップ目標値生成
            target_pos0, target_pos1 = generate_step_targets(elapsed_time)

            # 現在値取得
            current_pos0 = odrv0.axis0.pos_vel_mapper.pos_rel - initial_position0
            current_vel0 = odrv0.axis0.pos_vel_mapper.vel
            current_pos1 = odrv1.axis0.pos_vel_mapper.pos_rel - initial_position1
            current_vel1 = odrv1.axis0.pos_vel_mapper.vel
            output_pos = odrv2.axis0.pos_vel_mapper.pos_rel - initial_position2
            output_vel = odrv2.axis0.pos_vel_mapper.vel

            # 出力軸の目標値（ステップ応答に合わせる）
            output_target = target_pos0 + target_pos1  # 例：両モータの目標値合算（機構に応じて調整）

            # 出力軸PID
            tau_out_ref, _, _, _, _ = output_pid.update(output_target, output_pos)

            # ノルム最適化で2モータのトルク指令を計算
            total_torque0, total_torque1 = calc_min_norm_torque(tau_out_ref)

            # モータごとの位置誤差
            pos_error0 = target_pos0 - current_pos0
            pos_error1 = target_pos1 - current_pos1

            # トルク制限
            max_torque0 = SAFETY_CONFIG['max_torque0']
            max_torque1 = SAFETY_CONFIG['max_torque1']
            total_torque0 = max(min(total_torque0, max_torque0), -max_torque0)
            total_torque1 = max(min(total_torque1, max_torque1), -max_torque1)

            # モータ制御
            odrv0.axis0.controller.input_torque = total_torque0
            odrv1.axis0.controller.input_torque = total_torque1

            # データ記録（スレッドセーフ）
            with data_lock:
                data_log['time'].append(elapsed_time)
                data_log['motor0']['pos'].append(current_pos0)
                data_log['motor0']['vel'].append(current_vel0)
                data_log['motor0']['torque'].append(total_torque0)
                data_log['motor0']['setpoint_pos'].append(target_pos0)
                data_log['motor0']['setpoint_vel'].append(0)
                data_log['motor0']['error_pos'].append(pos_error0)
                data_log['motor0']['error_vel'].append(0)

                data_log['motor1']['pos'].append(current_pos1)
                data_log['motor1']['vel'].append(current_vel1)
                data_log['motor1']['torque'].append(total_torque1)
                data_log['motor1']['setpoint_pos'].append(target_pos1)
                data_log['motor1']['setpoint_vel'].append(0)
                data_log['motor1']['error_pos'].append(pos_error1)
                data_log['motor1']['error_vel'].append(0)

                data_log['output']['pos'].append(output_pos)
                data_log['output']['vel'].append(output_vel)

            loop_count += 1

            # ステータス表示
            if loop_count % 500 == 0:
                print(f"時間: {elapsed_time:.2f}s | M0: {current_pos0:.3f}pos {current_vel0:.3f}vel | M1: {current_pos1:.3f}pos {current_vel1:.3f}vel | Out: {output_pos:.3f}")
            
    except KeyboardInterrupt:
        print("\\n制御を停止中...")
        control_running = False
    
    finally:
        # モータ停止
        odrv0.axis0.controller.input_torque = 0
        odrv1.axis0.controller.input_torque = 0
        # モータアイドル化
        odrv0.axis0.requested_state = AxisState.IDLE
        odrv1.axis0.requested_state = AxisState.IDLE
        # データ保存
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = f'csv/integrated_pid_torque_{timestamp}.csv'
        
        # CSVファイル作成
        with open(csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'time', 
                'motor0_pos', 'motor0_vel', 'motor0_torque', 'motor0_setpoint_pos', 'motor0_setpoint_vel', 'motor0_error_pos', 'motor0_error_vel',
                'motor1_pos', 'motor1_vel', 'motor1_torque', 'motor1_setpoint_pos', 'motor1_setpoint_vel', 'motor1_error_pos', 'motor1_error_vel',
                'output_pos', 'output_vel'
            ])
            
            with data_lock:
                for i in range(len(data_log['time'])):
                    writer.writerow([
                        data_log['time'][i],
                        data_log['motor0']['pos'][i], data_log['motor0']['vel'][i], data_log['motor0']['torque'][i],
                        data_log['motor0']['setpoint_pos'][i], data_log['motor0']['setpoint_vel'][i], 
                        data_log['motor0']['error_pos'][i], data_log['motor0']['error_vel'][i],
                        data_log['motor1']['pos'][i], data_log['motor1']['vel'][i], data_log['motor1']['torque'][i],
                        data_log['motor1']['setpoint_pos'][i], data_log['motor1']['setpoint_vel'][i],
                        data_log['motor1']['error_pos'][i], data_log['motor1']['error_vel'][i],
                        data_log['output']['pos'][i], data_log['output']['vel'][i]
                    ])
        
        # ゲイン設定保存
        # gains_filename = save_gains_to_file()
        
        print(f"データ保存完了: {csv_filename}")
        # print(f"ゲイン設定保存: {gains_filename}")
        
        # ステップ応答解析とグラフ表示
        try:
            print("\n=== ステップ応答解析とグラフ表示 ===")
            final_graph_path, final_csv_path = analyze_and_plot_step_response(csv_filename)
            
            # 最終的な保存状況を報告
            print("\n=== 最終的なファイル状況 ===")
            if final_csv_path:
                print(f"📊 CSVデータ: {final_csv_path}")
            else:
                print("📊 CSVデータ: 削除済み")
            
            if final_graph_path:
                print(f"📈 グラフファイル: {final_graph_path}")
            else:
                print("📈 グラフファイル: 削除済み")
                
        except Exception as e:
            print(f"ステップ応答解析エラー: {e}")
            print("手動でCSVファイルを確認してください")
        

        
        print("制御終了")

if __name__ == "__main__":
    main()