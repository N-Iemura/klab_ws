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
error_queue1 = deque(maxlen=5)

# Define impulse parameters
impulse_time = 1.0  # Time after which to apply the impulse
impulse_duration = 0.005  # Duration of the impulse in seconds
impulse_position = 0.1  # Position to set during the impulse
loop_executed = False
second_impulse_executed = False
n = 10
impulse_flags = [0] * n

### leg ###
l1=0.45
l2=0.5
fx = 2

# Function to read position commands from a CSV file
# def read_position_commands(csv_file):
#     positions = []
#     with open(csv_file, 'r') as file:
#         reader = csv.reader(file)
#         next(reader)  # Skip header
#         for row in reader:
#             positions.append((-float(row[1])/360, -float(row[2])/360))  # Read Hip-Knee Angle and Knee-Ankle Angle
#     return positions

try:
    # 90度の位置を設定 (90度 = 0.25回転)
    target_position = 0.5  # 90度をターン単位で表現
    
    while True:
        # Calculate the elapsed time
        time_diff = time.time() - prev_time
        prev_time = time.time()
        elapsed_time = time.time() - start_time 
        
        # Current position
        current_pos0, current_pos1 = odrv0.axis0.pos_vel_mapper.pos_rel-initial_position0, odrv1.axis0.pos_vel_mapper.pos_rel-initial_position1

        # Set position for odrv0 and odrv1 to maintain 90 degrees
        odrv0.axis0.controller.input_pos = initial_position0
        odrv1.axis0.controller.input_pos = initial_position1 + target_position

        # Add the data to the list
        current_data_0.append(odrv0.axis0.motor.foc.Iq_measured)
        vel_data_0.append(odrv0.axis0.pos_vel_mapper.vel)
        position_data_0.append(current_pos0)
        current_data_1.append(odrv1.axis0.motor.foc.Iq_measured)
        vel_data_1.append(odrv1.axis0.pos_vel_mapper.vel)
        position_data_1.append(current_pos1)
        time_data.append(elapsed_time) 
        output_pos_data.append(odrv2.axis0.pos_vel_mapper.pos_rel)
        output_vel_data.append(odrv2.axis0.pos_vel_mapper.vel)

        # Break the loop if the elapsed time exceeds a certain limit (e.g., 10 seconds)
        if elapsed_time > 20:
            break

except KeyboardInterrupt:    
    now = datetime.now()
    # Format the date and time as a string
    timestamp = now.strftime("%Y%m%d_%H%M")
    # Create the filename
    filename = f'csv/two_pos_{timestamp}.csv'

    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'Velocity_0', 'Position_0', 'Velocity_1', 'Position_1', 'Output_pos_0', 'Output_vel_0', 'Current_0', 'Current_1'])
        # Write the data to the CSV file

        writer.writerows(zip(time_data, vel_data_0, position_data_0, vel_data_1, position_data_1, output_pos_data, output_vel_data, current_data_0, current_data_1))
    print(f"Data saved to {filename}")
    # Clear the data lists
    time_data.clear()
    time_d_data.clear()
    current_data_0.clear()
    vel_data_0.clear()
    position_data_0.clear()
    current_data_1.clear()
    vel_data_1.clear()
    position_data_1.clear()
    output_pos_data.clear()
    output_vel_data.clear()
    input_torque0.clear()
    input_torque1.clear()
    ref0.clear()
    ref1.clear()
    print("Program interrupted. Data saved and cleared.")
    # Clear errors
    odrv0.clear_errors()
    odrv1.clear_errors()
    # Reset the ODrive states
    odrv0.axis0.requested_state = AxisState.IDLE
    odrv1.axis0.requested_state = AxisState.IDLE