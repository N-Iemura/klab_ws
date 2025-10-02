import odrive
from odrive.enums import *
import time
import numpy as np
import csv
from datetime import datetime

# ODriveの初期化（シリアル番号は適宜変更）
odrv0 = odrive.find_any(serial_number='3856345D3539')  # Motor1
odrv1 = odrive.find_any(serial_number='384D346F3539')  # Motor2
odrv2 = odrive.find_any(serial_number='3849346F3539')  # Encoder (出力軸)

# 減速比
r1 = 1/20
r2 = 163/2000

# ムーア・ペンローズ擬似逆行列の計算
A = np.array([[r1, r2]])  # 1x2
A_plus = np.dot(A.T, np.linalg.inv(np.dot(A, A.T)))  # 2x1

# ODrive制御モード設定
odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
odrv0.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
odrv1.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
odrv1.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL

# データ保存用リスト
time_data = []
vel_data_0 = []
vel_data_1 = []
position_data_0 = []
position_data_1 = []
output_pos_data = []
output_vel_data = []
current_data_0 = []
current_data_1 = []

# 実験パラメータ
theta_dot_ref = 10.0  # 出力軸目標速度 [turn/s]
duration = 5.0        # 実験時間 [s]
dt = 0.05             # 制御周期 [s]

initial_position0 = odrv0.axis0.pos_vel_mapper.pos_rel
initial_position1 = odrv1.axis0.pos_vel_mapper.pos_rel

start_time = time.time()
try:
    while time.time() - start_time < duration:
        elapsed_time = time.time() - start_time

        # 最小ノルム解（z=0）
        u = np.dot(A_plus, theta_dot_ref)  # u = [theta_dot_1, theta_dot_2]
        theta_dot_1, theta_dot_2 = u[0], u[1]

        # ODriveへ速度指令
        odrv0.axis0.controller.input_vel = theta_dot_1
        odrv1.axis0.controller.input_vel = theta_dot_2

        # データ取得
        current_pos0 = odrv0.axis0.pos_vel_mapper.pos_rel - initial_position0
        current_pos1 = odrv1.axis0.pos_vel_mapper.pos_rel - initial_position1
        vel0 = odrv0.axis0.pos_vel_mapper.vel
        vel1 = odrv1.axis0.pos_vel_mapper.vel
        out_pos = odrv2.axis0.pos_vel_mapper.pos_rel
        out_vel = odrv2.axis0.pos_vel_mapper.vel
        iq0 = odrv0.axis0.motor.foc.Iq_measured
        iq1 = odrv1.axis0.motor.foc.Iq_measured

        # データ保存
        time_data.append(elapsed_time)
        vel_data_0.append(vel0)
        vel_data_1.append(vel1)
        position_data_0.append(current_pos0)
        position_data_1.append(current_pos1)
        output_pos_data.append(out_pos)
        output_vel_data.append(out_vel)
        current_data_0.append(iq0)
        current_data_1.append(iq1)

        time.sleep(dt)

except KeyboardInterrupt:
    pass

finally:
    now = datetime.now()
    timestamp = now.strftime("%Y%m%d_%H%M")
    filename = f'csv/unification_exp_{timestamp}.csv'
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'Velocity_0', 'Position_0', 'Velocity_1', 'Position_1', 'Output_pos', 'Output_vel', 'Current_0', 'Current_1'])
        writer.writerows(zip(time_data, vel_data_0, position_data_0, vel_data_1, position_data_1, output_pos_data, output_vel_data, current_data_0, current_data_1))
    print(f"Data saved to {filename}")

    odrv0.axis0.requested_state = AxisState.IDLE
    odrv1.axis0.requested_state = AxisState.IDLE