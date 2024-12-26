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

odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
odrv0.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
odrv0.axis0.config.motor.torque_constant = 0.106 #(トルク定数 8.23/Kv)
odrv0.axis0.controller.input_torque = 0

odrv1.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
odrv1.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
odrv1.axis0.config.motor.torque_constant = 0.106 #(トルク定数 8.23/Kv)
odrv1.axis0.controller.input_torque = 0

odrv0.axis0.controller.config.pos_gain = 70.0
odrv0.axis0.controller.config.vel_gain = 5

odrv1.axis0.controller.config.pos_gain = 70.0
odrv1.axis0.controller.config.vel_gain = 5

start_time = time.time()  # Get the current time
time1 = 0

## motor only
Kp0 = 2.8 # Proportional gain
Ki0 = 0.8  # Integral gain
Kd0 = 0.15 # Derivative gain
Kp1 = 2.5  # Proportional gain
Ki1 = 0.65 # Integral gain
Kd1 = 0.15 # Derivative gain
prev_error0 = 0
prev_error1 = 0
prev_time = time.time()
error_integral0 = 0
error_integral1 = 0

# 指令値を格納するリスト
ref0 = []
ref1 = []
error_queue0 = deque(maxlen=5)
error_queue1 = deque(maxlen=5)

# Define impulse parameters
impulse_time = 1.0  # Time after which to apply the impulse
impulse_duration = 0.005  # Duration of the impulse in seconds
impulse_position = 0.1  # Position to set during the impulse
loop_executed = False
second_impulse_executed = False

try:
    while True:
        # Calculate the elapsed time
        time_diff = time.time() - prev_time
        prev_time = time.time()
        elapsed_time = time.time() - start_time 
        
        # Current position
        current_pos0 = odrv0.axis0.pos_vel_mapper.pos_rel-initial_position0
        current_pos1 = odrv1.axis0.pos_vel_mapper.pos_rel-initial_position1

        # Add the data to the list
        current_data_0.append( math.sqrt(odrv0.axis0.motor.foc.Iq_measured**2 + odrv0.axis0.motor.foc.Id_measured**2))
        vel_data_0.append(360*math.pi*odrv0.axis0.pos_vel_mapper.vel/180)
        position_data_0.append(current_pos0)
        current_data_1.append( math.sqrt(odrv1.axis0.motor.foc.Iq_measured**2 + odrv1.axis0.motor.foc.Id_measured**2))
        vel_data_1.append(360*math.pi*odrv1.axis0.pos_vel_mapper.vel/180)
        position_data_1.append(current_pos1)
        time_data.append(elapsed_time)

        # Apply impulse position input at the specified time
        if loop_executed==False and 2+impulse_time <= elapsed_time:
            odrv0.axis0.controller.input_pos += impulse_position
            odrv1.axis0.controller.input_pos += impulse_position
            ref0.append(impulse_position)
            ref1.append(impulse_position)
            loop_executed = True
        elif loop_executed==True and not second_impulse_executed:
            # Apply second impulse position input at the specified time
            odrv0.axis0.controller.input_pos += impulse_position
            odrv1.axis0.controller.input_pos += impulse_position
            ref0.append(impulse_position)
            ref1.append(impulse_position)
            second_impulse_executed = True
        else:
            # Set the position input back to the desired position after the impulse
            odrv0.axis0.controller.input_pos = initial_position0
            odrv1.axis0.controller.input_pos = initial_position1
            ref0.append(0)
            ref1.append(0)
                
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
    odrv0.axis0.requested_state = AxisState.IDLE
    odrv1.axis0.requested_state = AxisState.IDLE
