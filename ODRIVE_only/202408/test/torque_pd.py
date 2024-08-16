import odrive
from odrive.enums import *
import time
import math
import matplotlib.pyplot as plt
import numpy as np
import csv

TORQUE_CONTROL= 1
MOTOR_CALIBRATION = 4
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
VELOCITY_CONTROL = 2

# Find two ODrives
# #maxon
# odrv0 = odrive.find_any(serial_number='385B34743539')
# #R100
odrv1 = odrive.find_any(serial_number='385E344A3539')

# odrv2 = odrive.find_any(serial_number='3849346F3539')

# odrv0.axis0.requested_state = MOTOR_CALIBRATION
# time.sleep(7) 
time_data = []
time_d_data = []
current_data_maxon = []
vel_data_maxon = []
position_data_maxon = []
current_data_r100 = []
vel_data_r100 = []
position_data_r100 = []
output_pos_data = []
output_vel_data = []

input_torque = []

# initial_position0 = odrv0.axis0.pos_vel_mapper.pos_rel
initial_position1 = odrv1.axis0.pos_vel_mapper.pos_rel
# initial_position2 = odrv2.axis0.pos_vel_mapper.pos_rel

# Set up each ODrive
# odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
# odrv0.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
# odrv0.axis0.controller.config.vel_ramp_rate = 3
# odrv0.axis0.controller.config.input_mode = InputMode.TUNING

odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv1.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
odrv1.axis0.config.motor.torque_constant = 8.23/105
odrv1.axis0.controller.input_torque = 0

start_time = time.time()  # Get the current time
time1 = 0
Kp = 0.03  # Proportional gain
Ki = 0.00001  # Integral gain
Kd = 0.000001  # Derivative gain
prev_error = 0
prev_time = time.time()
error_integral = 0

# try:
#     while True:
#         # Calculate the elapsed time
#         time_diff = time.time() - time1
#         time1 = time.time()
#         elapsed_time = time.time() - start_time
#         odrv1.axis0.controller.input_torque = 0.1*math.sin(elapsed_time)

#         input_torque.append(odrv1.axis0.controller.input_torque)
        

#         # Add the data to the lists for odrv0
#         current_data_maxon.append( math.sqrt(odrv1.axis0.motor.foc.Iq_measured**2 + odrv1.axis0.motor.foc.Id_measured**2))
#         vel_data_maxon.append(360*math.pi*odrv1.axis0.pos_vel_mapper.vel/180)
#         position_data_maxon.append(360*(odrv1.axis0.pos_vel_mapper.pos_rel-initial_position1))

#         time_data.append(elapsed_time)
#         time_d_data.append(time_diff)

try:
    while True:
        # Calculate the elapsed time
        time_diff = time.time() - prev_time
        prev_time = time.time()
        elapsed_time = time.time() - start_time
        print(f"Elapsed time for code block 1: {time_diff} seconds")
        # Current torque
        current_pos = odrv1.axis0.pos_vel_mapper.pos_rel-initial_position1

        desired_pos = 0.5
        print(f"Elapsed time for code block 2: {time_diff} seconds")

        # Calculate the error
        error = desired_pos - current_pos
        error_integral += error * time_diff
        print(f"Elapsed time for code block 3: {time_diff} seconds")

        # Calculate the derivative of the error
        if time_diff > 0:
            error_derivative = (error - prev_error) / time_diff
        else:
            error_derivative = 0
        print(f"Elapsed time for code block 4: {time_diff} seconds")

        # Calculate the new torque input
        new_torque = Kp * error + Kd * error_derivative + Ki * error_integral
        print(f"Elapsed time for code block 5: {time_diff} seconds")

        max_float32 = 3.4e38
        if new_torque > max_float32:
            new_torque = max_float32
        elif new_torque < -max_float32:
            new_torque = -max_float32
        
        print(f"Elapsed time for code block 6: {time_diff} seconds")

        # Set the new torque input
        odrv1.axis0.controller.input_torque = new_torque

        # Update the previous error
        prev_error = error

        input_torque.append(odrv1.axis0.controller.input_torque)
        print(f"Elapsed time for code block 7: {time_diff} seconds")

        # Add the data to the lists for odrv0
        current_data_maxon.append( math.sqrt(odrv1.axis0.motor.foc.Iq_measured**2 + odrv1.axis0.motor.foc.Id_measured**2))
        vel_data_maxon.append(360*math.pi*odrv1.axis0.pos_vel_mapper.vel/180)
        position_data_maxon.append(360*(odrv1.axis0.pos_vel_mapper.pos_rel-initial_position1))

        time_data.append(elapsed_time)
        print(f"Elapsed time for code block 8: {time_diff} seconds")
                
except KeyboardInterrupt:
    with open('csv/sin_t.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        # writer.writerow(['time','current_r100', 'Velocity_r100', 'Position_r100','current_maxon', 'Velocity_maxon', 'Position_maxon','output_pos','output_vel'])
        # writer.writerows(zip(time_data, current_data_r100, vel_data_r100, position_data_r100,current_data_maxon, vel_data_maxon, position_data_maxon,output_pos_data,output_vel_data))
        writer.writerow(['time','input_torque', 'Velocity_maxon', 'Position_maxon'])
        writer.writerows(zip(time_data, input_torque, vel_data_maxon, position_data_maxon))

    # Set velocity to 0 if the program is interrupted
    # odrv0.axis0.controller.input_vel = 0
    odrv1.axis0.controller.input_torque = 0

