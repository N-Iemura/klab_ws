import odrive
from odrive.enums import *
import time
import math
import matplotlib.pyplot as plt
import numpy as np
import csv
from datetime import datetime

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
odrv0.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL

odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv1.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL

start_time = time.time()  # Get the current time
time1 = 0
Kp = 0.1  # Proportional gain
Ki = 0.00001  # Integral gain
Kd = 0.000001  # Derivative gain
prev_error0 = 0
prev_error1 = 0
prev_time = time.time()
error_integral0 = 0
error_integral1 = 0


# odrv0.axis0.controller.input_pos = 0
# odrv1.axis0.controller.input_pos = 0


try:
    while True:
        # Calculate the elapsed time
        time_diff = time.time() - prev_time
        prev_time = time.time()
        elapsed_time = time.time() - start_time
        
        # Current position
        current_pos0 = odrv0.axis0.pos_vel_mapper.pos_rel-initial_position0
        current_pos1 = odrv1.axis0.pos_vel_mapper.pos_rel-initial_position1

        desired_pos0 = -0.01
        desired_pos1 = -0.01

        if elapsed_time > 3:
            odrv0.axis0.controller.input_pos = initial_position0 -0.1
            odrv1.axis0.controller.input_pos = initial_position1 -0.1
        # odrv0.axis0.controller.input_pos = initial_position0 -0.1 - 0.1*np.sin(5*elapsed_time)
        # odrv1.axis0.controller.input_pos = initial_position1 -0.1 - 0.1*np.sin(5*elapsed_time)
        # Calculate the error
        error0 = desired_pos0 - current_pos0
        error1 = desired_pos1 - current_pos1
        error_integral0 += error0 * time_diff
        error_integral1 += error1 * time_diff

        # Calculate the derivative of the error
        if time_diff > 0:
            error_derivative0 = (error0 - prev_error0) / time_diff
            error_derivative1 = (error1 - prev_error1) / time_diff
        else:
            error_derivative0 = 0
            error_derivative1 = 0
        
        # Calculate the new torque input
        new_torque0 = Kp * error0 + Kd * error_derivative0 + Ki * error_integral0
        new_torque1 = Kp * error1 + Kd * error_derivative1 + Ki * error_integral1
        
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

        # Update the previous error
        prev_error0 = error0
        prev_error1 = error1

        input_torque0.append(odrv0.axis0.controller.input_torque)
        input_torque1.append(odrv1.axis0.controller.input_torque)
        
        # Add the data to the list
        current_data_0.append( math.sqrt(odrv0.axis0.motor.foc.Iq_measured**2 + odrv0.axis0.motor.foc.Id_measured**2))
        vel_data_0.append(360*math.pi*odrv0.axis0.pos_vel_mapper.vel/180)
        position_data_0.append(360*(current_pos0))
        current_data_1.append( math.sqrt(odrv1.axis0.motor.foc.Iq_measured**2 + odrv1.axis0.motor.foc.Id_measured**2))
        vel_data_1.append(360*math.pi*odrv1.axis0.pos_vel_mapper.vel/180)
        position_data_1.append(360*(current_pos1))

        time_data.append(elapsed_time)
                
except KeyboardInterrupt:    
    now = datetime.now()
    # Format the date and time as a string
    timestamp = now.strftime("%Y%m%d_%H%M%S")
    # Create the filename
    filename = f'csv/two_pos_-0.1_{timestamp}.csv'

    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time','input_0', 'Velocity_0', 'Position_0', 'input_1', 'Velocity_1', 'Position_1'])
        writer.writerows(zip(time_data, input_torque0, vel_data_0, position_data_0, input_torque1, vel_data_1, position_data_1))

    # Set velocity to 0 if the program is interrupted
    # odrv0.axis0.controller.input_vel = 0
    odrv0.axis0.controller.input_pos = initial_position0
    odrv1.axis0.controller.input_pos = initial_position1

