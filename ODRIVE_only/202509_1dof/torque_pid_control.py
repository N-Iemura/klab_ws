import odrive
from odrive.enums import *
import time
import math
import matplotlib.pyplot as plt
import numpy as np
import csv
from datetime import datetime
from collections import deque

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
        
    def update(self, setpoint, feedback):
        current_time = time.time()
        dt = current_time - self.prev_time
        
        if dt <= 0.0:
            dt = 1e-6  # 最小時間間隔
            
        error = setpoint - feedback
        
        # 比例項
        proportional = self.kp * error
        
        # 積分項
        self.integral += error * dt
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
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()
        
    def update_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

# ODrive接続
print("ODriveを検索中...")
odrv0 = odrive.find_any(serial_number='3856345D3539')  # T-motor
odrv1 = odrive.find_any(serial_number='384D346F3539')  # Maxon  
odrv2 = odrive.find_any(serial_number='3849346F3539')  # Encoder
print("ODrive接続完了")

# データ記録用リスト
time_data = []
# Motor 0
position_data_0 = []
velocity_data_0 = []
torque_data_0 = []
setpoint_pos_0 = []
setpoint_vel_0 = []
error_pos_0 = []
error_vel_0 = []
# Motor 1
position_data_1 = []
velocity_data_1 = []
torque_data_1 = []
setpoint_pos_1 = []
setpoint_vel_1 = []
error_pos_1 = []
error_vel_1 = []
# Output
output_pos_data = []
output_vel_data = []

# 初期位置の記録
initial_position0 = odrv0.axis0.pos_vel_mapper.pos_rel
initial_position1 = odrv1.axis0.pos_vel_mapper.pos_rel
initial_position2 = odrv2.axis0.pos_vel_mapper.pos_rel

# モータ設定（トルク制御モード）
print("モータをトルク制御モードに設定中...")
odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
odrv0.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
odrv0.axis0.config.motor.torque_constant = 0.106  # T-motor トルク定数

odrv1.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
odrv1.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
odrv1.axis0.config.motor.torque_constant = 0.091  # Maxon トルク定数
print("モータ設定完了")

# PIDコントローラの初期化（調整可能なパラメータ）
print("PIDコントローラを初期化中...")

# Motor 0 PIDゲイン（位置制御）
motor0_pos_kp = 20.0   # 位置比例ゲイン
motor0_pos_ki = 0.1    # 位置積分ゲイン
motor0_pos_kd = 2.0    # 位置微分ゲイン

# Motor 0 PIDゲイン（速度制御）
motor0_vel_kp = 0.5    # 速度比例ゲイン
motor0_vel_ki = 0.05   # 速度積分ゲイン
motor0_vel_kd = 0.01   # 速度微分ゲイン

# Motor 1 PIDゲイン（位置制御）
motor1_pos_kp = 18.0   # 位置比例ゲイン
motor1_pos_ki = 0.08   # 位置積分ゲイン
motor1_pos_kd = 1.8    # 位置微分ゲイン

# Motor 1 PIDゲイン（速度制御）
motor1_vel_kp = 0.4    # 速度比例ゲイン
motor1_vel_ki = 0.04   # 速度積分ゲイン
motor1_vel_kd = 0.008  # 速度微分ゲイン

# PIDコントローラインスタンス作成
pid_pos_0 = PIDController(motor0_pos_kp, motor0_pos_ki, motor0_pos_kd, max_output=5.0)
pid_vel_0 = PIDController(motor0_vel_kp, motor0_vel_ki, motor0_vel_kd, max_output=2.0)
pid_pos_1 = PIDController(motor1_pos_kp, motor1_pos_ki, motor1_pos_kd, max_output=5.0)
pid_vel_1 = PIDController(motor1_vel_kp, motor1_vel_ki, motor1_vel_kd, max_output=2.0)

print("PIDコントローラ初期化完了")

# 制御パラメータ
start_time = time.time()
control_frequency = 1000  # 制御周波数 [Hz]
dt_target = 1.0 / control_frequency

# 目標値生成パラメータ
position_amplitude = 0.5  # 位置指令の振幅 [turn]
velocity_amplitude = 2.0  # 速度指令の振幅 [turn/s]
frequency = 0.2          # 指令周波数 [Hz]

print("=== トルク制御PID環境準備完了 ===")
print(f"Motor 0 位置PID: Kp={motor0_pos_kp}, Ki={motor0_pos_ki}, Kd={motor0_pos_kd}")
print(f"Motor 0 速度PID: Kp={motor0_vel_kp}, Ki={motor0_vel_ki}, Kd={motor0_vel_kd}")
print(f"Motor 1 位置PID: Kp={motor1_pos_kp}, Ki={motor1_pos_ki}, Kd={motor1_pos_kd}")
print(f"Motor 1 速度PID: Kp={motor1_vel_kp}, Ki={motor1_vel_ki}, Kd={motor1_vel_kd}")
print("Ctrl+Cで終了してください")

try:
    loop_count = 0
    
    while True:
        loop_start_time = time.time()
        elapsed_time = loop_start_time - start_time
        
        # 現在の位置・速度を取得
        current_pos0 = odrv0.axis0.pos_vel_mapper.pos_rel - initial_position0
        current_vel0 = odrv0.axis0.pos_vel_mapper.vel
        current_pos1 = odrv1.axis0.pos_vel_mapper.pos_rel - initial_position1
        current_vel1 = odrv1.axis0.pos_vel_mapper.vel
        output_pos = odrv2.axis0.pos_vel_mapper.pos_rel - initial_position2
        output_vel = odrv2.axis0.pos_vel_mapper.vel
        
        # 目標値生成（正弦波）
        target_pos0 = position_amplitude * math.sin(2 * math.pi * frequency * elapsed_time)
        target_vel0 = velocity_amplitude * math.sin(2 * math.pi * frequency * elapsed_time + math.pi/4)
        
        target_pos1 = position_amplitude * math.sin(2 * math.pi * frequency * elapsed_time + math.pi)
        target_vel1 = velocity_amplitude * math.sin(2 * math.pi * frequency * elapsed_time + math.pi + math.pi/4)
        
        # Motor 0 PID制御
        pos_torque0, pos_error0, _, _, _ = pid_pos_0.update(target_pos0, current_pos0)
        vel_torque0, vel_error0, _, _, _ = pid_vel_0.update(target_vel0, current_vel0)
        total_torque0 = pos_torque0 + vel_torque0
        
        # Motor 1 PID制御
        pos_torque1, pos_error1, _, _, _ = pid_pos_1.update(target_pos1, current_pos1)
        vel_torque1, vel_error1, _, _, _ = pid_vel_1.update(target_vel1, current_vel1)
        total_torque1 = pos_torque1 + vel_torque1
        
        # トルク制限
        max_torque = 3.0  # 最大トルク制限
        total_torque0 = max(min(total_torque0, max_torque), -max_torque)
        total_torque1 = max(min(total_torque1, max_torque), -max_torque)
        
        # モータにトルク指令を送信
        odrv0.axis0.controller.input_torque = total_torque0
        odrv1.axis0.controller.input_torque = total_torque1
        
        # データ記録
        time_data.append(elapsed_time)
        
        # Motor 0 データ
        position_data_0.append(current_pos0)
        velocity_data_0.append(current_vel0)
        torque_data_0.append(total_torque0)
        setpoint_pos_0.append(target_pos0)
        setpoint_vel_0.append(target_vel0)
        error_pos_0.append(pos_error0)
        error_vel_0.append(vel_error0)
        
        # Motor 1 データ
        position_data_1.append(current_pos1)
        velocity_data_1.append(current_vel1)
        torque_data_1.append(total_torque1)
        setpoint_pos_1.append(target_pos1)
        setpoint_vel_1.append(target_vel1)
        error_pos_1.append(pos_error1)
        error_vel_1.append(vel_error1)
        
        # Output データ
        output_pos_data.append(output_pos)
        output_vel_data.append(output_vel)
        
        loop_count += 1
        
        # 制御周期の調整
        loop_duration = time.time() - loop_start_time
        if loop_duration < dt_target:
            time.sleep(dt_target - loop_duration)
            
        # 1秒ごとに状態を表示
        if loop_count % 1000 == 0:
            print(f"時間: {elapsed_time:.2f}s, M0位置: {current_pos0:.3f}, M1位置: {current_pos1:.3f}, 出力: {output_pos:.3f}")

except KeyboardInterrupt:
    print("\n制御を停止しています...")
    
    # モータ停止
    odrv0.axis0.controller.input_torque = 0
    odrv1.axis0.controller.input_torque = 0
    
    # データ保存
    now = datetime.now()
    timestamp = now.strftime("%Y%m%d_%H%M%S")
    filename = f'csv/pid_torque_control_{timestamp}.csv'
    
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 
            'pos_0', 'vel_0', 'torque_0', 'setpoint_pos_0', 'setpoint_vel_0', 'error_pos_0', 'error_vel_0',
            'pos_1', 'vel_1', 'torque_1', 'setpoint_pos_1', 'setpoint_vel_1', 'error_pos_1', 'error_vel_1',
            'output_pos', 'output_vel'
        ])
        
        writer.writerows(zip(
            time_data,
            position_data_0, velocity_data_0, torque_data_0, setpoint_pos_0, setpoint_vel_0, error_pos_0, error_vel_0,
            position_data_1, velocity_data_1, torque_data_1, setpoint_pos_1, setpoint_vel_1, error_pos_1, error_vel_1,
            output_pos_data, output_vel_data
        ))
        
    print(f"データを保存しました: {filename}")
    
    # モータをアイドル状態に
    odrv0.axis0.requested_state = AxisState.IDLE
    odrv1.axis0.requested_state = AxisState.IDLE
    
    print("制御終了")