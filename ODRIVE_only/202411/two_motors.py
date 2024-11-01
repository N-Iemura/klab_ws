import odrive
from odrive.enums import *
import time
import math
import matplotlib.pyplot as plt
import numpy as np
import csv
from datetime import datetime
from collections import deque

TORQUE_CONTROL= 1
MOTOR_CALIBRATION = 4
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
VELOCITY_CONTROL = 2

# Find two ODrives
# #0
odrv0 = odrive.find_any(serial_number='385B34743539')
# #1    
odrv1 = odrive.find_any(serial_number='385E344A3539')

# odrv2 = odrive.find_any(serial_number='3849346F3539')

# odrv0.axis0.requested_state = MOTOR_CALIBRATION
# time.sleep(7) 
time_data = []
time_d_data = []
current_data_0= []
vel_data_0 = []
position_data_0 = []
current_data_1 = []
vel_data_1 = []
position_data_1 = []
output_pos_data0 = []
output_vel_data0 = []
output_pos_data1 = []
output_vel_data1 = []

input_torque0 = []
input_torque1 = []

initial_position0 = odrv0.axis0.pos_vel_mapper.pos_rel
initial_position1 = odrv1.axis0.pos_vel_mapper.pos_rel
# initial_position2 = odrv2.axis0.pos_vel_mapper.pos_rel

odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
odrv0.axis0.config.motor.torque_constant = 0.106 #(トルク定数 8.23/Kv)
odrv0.axis0.controller.input_torque = 0

odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv1.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
odrv1.axis0.config.motor.torque_constant = 0.106 #(トルク定数 8.23/Kv)
odrv1.axis0.controller.input_torque = 0

start_time = time.time()  # Get the current time
time1 = 0
## motor with link
Kp0 = 200 # Proportional gain
Ki0 = 200  # Integral gain
Kd0 = 0  # Derivative gain
Kp1 = 300 # Proportional gain
Ki1 = 200 # Integral gain
Kd1 = 10 # Derivative gain

## motor only
# Kp0 = 2.8 # Proportional gain
# Ki0 = 0.8  # Integral gain
# Kd0 = 0.15 # Derivative gain
# Kp1 = 2.5  # Proportional gain
# Ki1 = 0.65 # Integral gain
# Kd1 = 0.15 # Derivative gain
prev_error0 = 0
prev_error1 = 0
prev_time = time.time()
error_integral0 = 0
error_integral1 = 0

# アンチワインドアップのための積分項の制限
integral_limit = 600
Ti = 0.5  # 時定数

# ローパスフィルタの初期化
filtered_desired_pos0 = 0
filtered_desired_pos1 = 0

# 指令値を格納するリスト
ref0 = []
ref1 = []
error_queue0 = deque(maxlen=10)
error_queue1 = deque(maxlen=10)

try:
    while True:
        # Calculate the elapsed time
        time_diff = time.time() - prev_time
        prev_time = time.time()
        elapsed_time = time.time() - start_time 
        
        # Current position
        current_pos0 = odrv0.axis0.pos_vel_mapper.pos_rel-initial_position0
        current_pos1 = odrv1.axis0.pos_vel_mapper.pos_rel-initial_position1


        if elapsed_time > 0.5:
            desired_pos0 = -0.1
            desired_pos1 = -0.1
            # desired_pos0 = -0.2*np.sin(5*elapsed_time)
            # desired_pos1 = -0.2*np.sin(5*elapsed_time)
            filtered_desired_pos0 = Ti/(Ti+time_diff)*filtered_desired_pos0 + time_diff/(Ti+time_diff)*desired_pos0
            filtered_desired_pos1 = Ti/(Ti+time_diff)*filtered_desired_pos1 + time_diff/(Ti+time_diff)*desired_pos1
            
            # 指令値をリストに格納
            ref0.append(filtered_desired_pos0)
            ref1.append(filtered_desired_pos1)
            
             
            # Calculate the error - initial_position0
            # 誤差の計算
            error0 = filtered_desired_pos0 - current_pos0
            error1 = filtered_desired_pos1 - current_pos1
            # 誤差をデックに追加
            error_queue0.append(error0)
            error_queue1.append(error1)
            # error_integral0 = max(min(error_integral0, integral_limit), -integral_limit)
            # error_integral1 = max(min(error_integral1, integral_limit), -integral_limit)
            error_integral0 += error0 * time_diff
            error_integral1 += error1 * time_diff

            # 誤差の微分項の計算
            if len(error_queue0) == 10:
                error_derivative0 = (error_queue0[-1] - error_queue0[0]) / (10 * time_diff)
            else:
                error_derivative0 = 0

            if len(error_queue1) == 10:
                error_derivative1 = (error_queue1[-1] - error_queue1[0]) / (10 * time_diff)
            else:
                error_derivative1 = 0
            
            # Calculate the new torque input
            new_torque0 = Kp0 * error0 + Kd0 * error_derivative0 + Ki0 * error_integral0
            new_torque1 = Kp1 * error1 + Kd1 * error_derivative1 + Ki1 * error_integral1

            max_float32 = 3.4e38
            if new_torque0 > max_float32:
                new_torque0 = max_float32
            elif new_torque0 < -max_float32:    
                new_torque0 = -max_float32
            if new_torque1 > max_float32:
                new_torque1 = max_float32
            elif new_torque1 < -max_float32:
                new_torque1 = -max_float32
            
            # Set the new torque input
            odrv0.axis0.controller.input_torque = new_torque0
            odrv1.axis0.controller.input_torque = new_torque1

            # Update the previous error
            prev_error0 = error0
            prev_error1 = error1

            input_torque0.append(odrv0.axis0.controller.input_torque)
            input_torque1.append(odrv1.axis0.controller.input_torque)
        else:
            desired_pos0 = 0
            desired_pos1 = 0
            filtered_desired_pos0 = 0
            filtered_desired_pos1 = 0
            input_torque0.append(odrv0.axis0.controller.input_torque)
            input_torque1.append(odrv1.axis0.controller.input_torque)
            ref0.append(filtered_desired_pos0)
            ref1.append(filtered_desired_pos1)

        
        # Add the data to the list
        current_data_0.append( math.sqrt(odrv0.axis0.motor.foc.Iq_measured**2 + odrv0.axis0.motor.foc.Id_measured**2))
        vel_data_0.append(360*math.pi*odrv0.axis0.pos_vel_mapper.vel/180)
        position_data_0.append(current_pos0)
        current_data_1.append( math.sqrt(odrv1.axis0.motor.foc.Iq_measured**2 + odrv1.axis0.motor.foc.Id_measured**2))
        vel_data_1.append(360*math.pi*odrv1.axis0.pos_vel_mapper.vel/180)
        position_data_1.append(current_pos1)

        time_data.append(elapsed_time)
        print("pos0: ", current_pos0)
        print("pos1: ", current_pos1)
                
except KeyboardInterrupt:    
    now = datetime.now()
    # Format the date and time as a string
    timestamp = now.strftime("%Y%m%d_%H%M%S")
    # Create the filename
    filename = f'csv/two_pos_-0.1_{timestamp}.csv'

    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time','ref_0', 'Velocity_0', 'Position_0', 'ref_1', 'Velocity_1', 'Position_1'])
        writer.writerows(zip(time_data, ref0, vel_data_0, position_data_0, ref1, vel_data_1, position_data_1))

    # Set velocity to 0 if the program is interrupted
    # odrv0.axis0.controller.input_vel = 0
    odrv0.axis0.controller.input_torque = 0
    odrv1.axis0.controller.input_torque = 0

