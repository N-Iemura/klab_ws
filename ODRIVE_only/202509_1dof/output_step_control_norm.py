"""
出力軸ステップ制御 - ノルム最小化版
可変減速機構の出力軸を指令し、ノルム最小化によりMotor0,1の目標値を算出

可変減速機構の原理:
- output_pos = motor0_pos * (1/20) + motor1_pos * (163/2000)
- 出力目標値が与えられた時、2入力1出力のため解は無数に存在
- ノルム最小化により一意解を求める: min ||[motor0_pos, motor1_pos]||²

使用方法:
1. python output_step_control_norm.py を実行
2. 別ターミナルで python pid_gain_adjuster_integrated.py を実行してリアルタイム調整

特徴:
- 出力軸位置を直接指令
- ノルム最小化による最適なモータ配分
- 両モータのトルク制御による精密制御
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
from scipy.optimize import minimize

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
# 出力軸制御設定 - ここを編集してステップ応答パターンをカスタマイズ
# ==================================================================================

# ステップ応答パターン設定
STEP_CONFIG = {
    # タイミング設定
    'initial_wait': 1.0,        # 最初のステップまでの待機時間[秒]
    'step_duration': 3.0,       # 各ステップの持続時間[秒]
    
    # 出力軸のステップ振幅[turn]
    'output_amplitude': 0.2,    # 出力軸のステップ高さ
    
    # 可変減速機構パラメータ
    'motor0_gear_ratio': 1/20,      # Motor0の減速比 (1/20)
    'motor1_gear_ratio': 163/2000,  # Motor1の減速比 (163/2000)
    
    # ノルム最小化設定
    'use_weighted_norm': True,      # 重み付きノルムを使用するか
    'motor0_weight': 1.0,           # Motor0の重み（T-motor）
    'motor1_weight': 10.0,          # Motor1の重み（Maxon、振動対策で大きく）
}

# PIDゲイン設定（各モータ制御用）
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
    
    os.makedirs('gain', exist_ok=True)
    filename = f"gain/norm_control_gains_{int(time.time())}.json"
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

def calculate_motor_targets_norm_minimization(target_output_pos):
    """
    ノルム最小化により出力目標値からモータ目標値を算出
    
    問題設定:
    minimize: ||W * [motor0_pos, motor1_pos]||²
    subject to: target_output_pos = motor0_pos * (1/20) + motor1_pos * (163/2000)
    
    解析解:
    A = [1/20, 163/2000]  (係数ベクトル)
    W = diag([w0, w1])    (重み行列)
    
    解: x = W^(-2) * A^T * (A * W^(-2) * A^T)^(-1) * target_output_pos
    """
    
    # 可変減速機構の係数
    a0 = STEP_CONFIG['motor0_gear_ratio']  # 1/20
    a1 = STEP_CONFIG['motor1_gear_ratio']  # 163/2000
    A = np.array([a0, a1])
    
    if STEP_CONFIG['use_weighted_norm']:
        # 重み付きノルム最小化
        w0 = STEP_CONFIG['motor0_weight']
        w1 = STEP_CONFIG['motor1_weight']
        
        # W^(-2) = diag([1/w0^2, 1/w1^2])
        W_inv_sq = np.array([1/(w0**2), 1/(w1**2)])
        
        # A * W^(-2) * A^T (スカラー)
        denominator = np.dot(A, W_inv_sq * A)
        
        # 解: x = W^(-2) * A^T * (A * W^(-2) * A^T)^(-1) * target_output_pos
        motor_targets = (W_inv_sq * A * target_output_pos) / denominator
        
    else:
        # 通常のノルム最小化
        # 解: x = A^T * (A * A^T)^(-1) * target_output_pos
        denominator = np.dot(A, A)  # ||A||²
        motor_targets = A * target_output_pos / denominator
    
    target_motor0_pos = motor_targets[0]
    target_motor1_pos = motor_targets[1]
    
    return target_motor0_pos, target_motor1_pos

def generate_output_step_targets(elapsed_time):
    """出力軸のステップ目標値を生成"""
    initial_wait = STEP_CONFIG['initial_wait']
    step_duration = STEP_CONFIG['step_duration']
    
    # 初期待機時間
    if elapsed_time < initial_wait:
        return 0.0
    
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
    
    # 出力軸の目標値
    target_output_pos = step_value * STEP_CONFIG['output_amplitude']
    
    return target_output_pos

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

def analyze_and_plot_step_response(csv_filename):
    """CSVデータからステップ応答を解析してグラフを表示"""
    print(f"ステップ応答解析を開始: {csv_filename}")
    
    # CSVファイル読み込み
    df = pd.read_csv(csv_filename)
    
    # グラフ作成（3x3で出力軸解析も追加）
    fig, axes = plt.subplots(3, 3, figsize=(18, 12))
    fig.suptitle('Norm Minimization Output Control - Step Response Analysis', fontsize=16)
    
    # Motor 0 の解析
    analyze_motor_response(df, 0, axes[:, 0])
    
    # Motor 1 の解析
    analyze_motor_response(df, 1, axes[:, 1])
    
    # 出力軸の解析
    time_data = df['time'].values
    
    # 出力位置応答
    axes[0, 2].plot(time_data, df['output_setpoint_pos'].values, 'r--', label='Target', linewidth=2)
    axes[0, 2].plot(time_data, df['output_pos'].values, 'b-', label='Actual', linewidth=1)
    axes[0, 2].set_title('Output - Position Step Response')
    axes[0, 2].set_ylabel('Position [turn]')
    axes[0, 2].grid(True)
    axes[0, 2].legend()
    
    # 出力位置エラー
    axes[1, 2].plot(time_data, df['output_error_pos'].values, 'r-', linewidth=1)
    axes[1, 2].set_title('Output - Position Error')
    axes[1, 2].set_ylabel('Error [turn]')
    axes[1, 2].grid(True)
    
    # モータ目標値の比較
    axes[2, 2].plot(time_data, df['motor0_setpoint_pos'].values, 'b-', linewidth=1, label='Motor0 Target')
    axes[2, 2].plot(time_data, df['motor1_setpoint_pos'].values, 'orange', linewidth=1, label='Motor1 Target')
    axes[2, 2].set_title('Motor Target Positions (Norm Minimized)')
    axes[2, 2].set_ylabel('Position [turn]')
    axes[2, 2].set_xlabel('Time [s]')
    axes[2, 2].grid(True)
    axes[2, 2].legend()
    
    plt.tight_layout()
    
    # 一時的にグラフファイルを作成（表示用）
    os.makedirs('fig', exist_ok=True)
    base_filename = os.path.basename(csv_filename).replace('.csv', '_norm_step_response.pdf')
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

def print_current_config():
    """現在の設定を表示"""
    print("\n=== 現在の制御設定 ===")
    print(f"出力軸ステップ振幅: {STEP_CONFIG['output_amplitude']} turn")
    print(f"ステップ持続時間: {STEP_CONFIG['step_duration']} 秒")
    print(f"初期待機時間: {STEP_CONFIG['initial_wait']} 秒")
    
    print("\n=== 可変減速機構設定 ===")
    print(f"減速機構の式: output_pos = motor0_pos * (1/20) + motor1_pos * (163/2000)")
    print(f"Motor0減速比: {STEP_CONFIG['motor0_gear_ratio']:.3f} (1/20)")
    print(f"Motor1減速比: {STEP_CONFIG['motor1_gear_ratio']:.6f} (163/2000)")
    
    print("\n=== ノルム最小化設定 ===")
    if STEP_CONFIG['use_weighted_norm']:
        print(f"ノルム最小化: 重み付き (Motor0重み: {STEP_CONFIG['motor0_weight']}, Motor1重み: {STEP_CONFIG['motor1_weight']})")
        print(f"目的関数: min ||W * [motor0_pos, motor1_pos]||²")
    else:
        print(f"ノルム最小化: 通常 (重み無し)")
        print(f"目的関数: min ||[motor0_pos, motor1_pos]||²")
    
    print("\n=== PIDゲイン設定 ===")
    for motor, config in PID_CONFIG.items():
        print(f"{motor}: kp={config['kp']}, ki={config['ki']}, kd={config['kd']}, max_output={config['max_output']}")
    
    print("\n=== 安全制限設定 ===")
    print(f"Motor0 最大トルク: {SAFETY_CONFIG['max_torque0']} Nm")
    print(f"Motor1 最大トルク: {SAFETY_CONFIG['max_torque1']} Nm")
    print("=" * 50)

def main():
    global pid_controllers, control_running
    
    print("=== 出力軸ステップ制御 - ノルム最小化版 ===")
    
    # 現在の設定を表示
    print_current_config()
    
    # ノルム最小化の動作例を表示
    print("\n=== ノルム最小化動作例 ===")
    test_output = 0.1
    test_motor0, test_motor1 = calculate_motor_targets_norm_minimization(test_output)
    print(f"出力目標 {test_output} turn の場合:")
    print(f"  Motor0目標: {test_motor0:.6f} turn")
    print(f"  Motor1目標: {test_motor1:.6f} turn")
    print(f"  検証: {test_motor0 * STEP_CONFIG['motor0_gear_ratio'] + test_motor1 * STEP_CONFIG['motor1_gear_ratio']:.6f} turn")
    
    # ODrive接続
    print("\nODriveを検索中...")
    try:
        odrv0 = odrive.find_any(serial_number='3856345D3539')  # T-motor
        odrv1 = odrive.find_any(serial_number='384D346F3539')  # Maxon
        odrv2 = odrive.find_any(serial_number='3849346F3539')  # Encoder (出力軸)
        print("ODrive接続完了")
    except Exception as e:
        print(f"ODrive接続エラー: {e}")
        return
    
    # 初期位置の記録
    initial_position0 = odrv0.axis0.pos_vel_mapper.pos_rel
    initial_position1 = odrv1.axis0.pos_vel_mapper.pos_rel
    initial_output_position = odrv2.axis0.pos_vel_mapper.pos_rel
    
    # モータ設定
    print("モータをトルク制御モードに設定中...")
    # Motor0: トルク制御モード
    odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
    odrv0.axis0.config.motor.torque_constant = 0.106
    
    # Motor1: トルク制御モード
    odrv1.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    odrv1.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
    odrv1.axis0.config.motor.torque_constant = 0.091
    
    print("モータ設定完了")
    
    # PIDコントローラ初期化
    print("PIDコントローラを初期化中...")
    
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
        'motor0': {'pos': [], 'vel': [], 'torque': [], 'setpoint_pos': [], 'error_pos': []},
        'motor1': {'pos': [], 'vel': [], 'torque': [], 'setpoint_pos': [], 'error_pos': []},
        'output': {'pos': [], 'vel': [], 'setpoint_pos': [], 'error_pos': []}
    }
    
    # 制御パラメータ
    start_time = time.time()
    control_frequency = 200  # Hz
    dt_target = 1.0 / control_frequency
    
    print("=== 制御開始 ===")
    print("出力軸位置を指令し、ノルム最小化によりモータ配分を決定します")
    print("リアルタイムゲイン調整: python pid_gain_adjuster_integrated.py")
    print("Ctrl+Cで終了")
    
    control_running = True
    loop_count = 0
    
    try:
        while control_running:
            loop_start_time = time.time()
            elapsed_time = loop_start_time - start_time
            
            # 現在値取得
            current_pos0 = odrv0.axis0.pos_vel_mapper.pos_rel - initial_position0
            current_vel0 = odrv0.axis0.pos_vel_mapper.vel
            current_pos1 = odrv1.axis0.pos_vel_mapper.pos_rel - initial_position1
            current_vel1 = odrv1.axis0.pos_vel_mapper.vel
            current_output_pos = odrv2.axis0.pos_vel_mapper.pos_rel - initial_output_position
            current_output_vel = odrv2.axis0.pos_vel_mapper.vel
            
            # 出力軸の目標値を生成
            target_output_pos = generate_output_step_targets(elapsed_time)
            
            # ノルム最小化によりモータ目標値を算出
            target_pos0, target_pos1 = calculate_motor_targets_norm_minimization(target_output_pos)
            
            # 可変減速機構の式に基づいて実際の出力軸位置を計算
            calculated_output_pos = current_pos0 * STEP_CONFIG['motor0_gear_ratio'] + current_pos1 * STEP_CONFIG['motor1_gear_ratio']
            
            # PID制御計算（両モータ）
            total_torque0, pos_error0, _, _, _ = pid_controllers['Motor0_Position'].update(target_pos0, current_pos0)
            total_torque1, pos_error1, _, _, _ = pid_controllers['Motor1_Position'].update(target_pos1, current_pos1)
            
            # 出力軸のエラー計算
            output_error = target_output_pos - calculated_output_pos
            
            # トルク制限（設定ファイルから読み込み）
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
                data_log['motor0']['error_pos'].append(pos_error0)
                
                data_log['motor1']['pos'].append(current_pos1)
                data_log['motor1']['vel'].append(current_vel1)
                data_log['motor1']['torque'].append(total_torque1)
                data_log['motor1']['setpoint_pos'].append(target_pos1)
                data_log['motor1']['error_pos'].append(pos_error1)
                
                data_log['output']['pos'].append(calculated_output_pos)
                data_log['output']['vel'].append(current_output_vel)
                data_log['output']['setpoint_pos'].append(target_output_pos)
                data_log['output']['error_pos'].append(output_error)
            
            loop_count += 1
            
            # ステータス表示
            if loop_count % 500 == 0:
                print(f"時間: {elapsed_time:.2f}s | 出力: {calculated_output_pos:.4f}pos(目標:{target_output_pos:.4f}) | M0: {current_pos0:.4f}(目標:{target_pos0:.4f}) | M1: {current_pos1:.4f}(目標:{target_pos1:.4f})")
            
            # 制御周波数の維持
            loop_end_time = time.time()
            sleep_time = dt_target - (loop_end_time - loop_start_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\n制御を停止中...")
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
        os.makedirs('csv', exist_ok=True)
        csv_filename = f'csv/norm_minimized_output_control_{timestamp}.csv'
        
        # CSVファイル作成
        with open(csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'time', 
                'motor0_pos', 'motor0_vel', 'motor0_torque', 'motor0_setpoint_pos', 'motor0_setpoint_vel', 'motor0_error_pos', 'motor0_error_vel',
                'motor1_pos', 'motor1_vel', 'motor1_torque', 'motor1_setpoint_pos', 'motor1_setpoint_vel', 'motor1_error_pos', 'motor1_error_vel',
                'output_pos', 'output_vel', 'output_setpoint_pos', 'output_error_pos'
            ])
            
            with data_lock:
                for i in range(len(data_log['time'])):
                    writer.writerow([
                        data_log['time'][i],
                        data_log['motor0']['pos'][i], data_log['motor0']['vel'][i], data_log['motor0']['torque'][i],
                        data_log['motor0']['setpoint_pos'][i], 0, data_log['motor0']['error_pos'][i], 0,
                        data_log['motor1']['pos'][i], data_log['motor1']['vel'][i], data_log['motor1']['torque'][i],
                        data_log['motor1']['setpoint_pos'][i], 0, data_log['motor1']['error_pos'][i], 0,
                        data_log['output']['pos'][i], data_log['output']['vel'][i], 
                        data_log['output']['setpoint_pos'][i], data_log['output']['error_pos'][i]
                    ])
        
        # ゲイン設定保存
        gains_filename = save_gains_to_file()
        
        print(f"データ保存完了: {csv_filename}")
        print(f"ゲイン設定保存: {gains_filename}")
        
        # ステップ応答解析とグラフ表示
        try:
            print("\n=== ノルム最小化ステップ応答解析とグラフ表示 ===")
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