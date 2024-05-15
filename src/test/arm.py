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
#maxon
odrv0 = odrive.find_any(serial_number='385B34743539')
#R100
odrv1 = odrive.find_any(serial_number='385E344A3539')

odrv2 = odrive.find_any(serial_number='3849346F3539')

# odrv0.axis0.requested_state = MOTOR_CALIBRATION
# time.sleep(7) 
time_data = []
current_data_maxon = []
vel_data_maxon = []
position_data_maxon = []
current_data_r100 = []
vel_data_r100 = []
position_data_r100 = []
output_pos_data = []
output_vel_data = []

initial_position0 = odrv0.axis0.pos_vel_mapper.pos_rel
initial_position1 = odrv1.axis0.pos_vel_mapper.pos_rel
initial_position2 = odrv2.axis0.pos_vel_mapper.pos_rel

odrive.utils.start_liveplotter(lambda: [
    #1
    360*(odrv0.axis0.pos_vel_mapper.pos_rel - initial_position0) , # Position value
    #2
    360*math.pi*odrv0.axis0.pos_vel_mapper.vel/180, # turns/s to rad/s
    #3
    math.sqrt(odrv0.axis0.motor.foc.Iq_measured**2 + odrv0.axis0.motor.foc.Id_measured**2),  # Current value
    #
    360*(odrv1.axis0.pos_vel_mapper.pos_rel - initial_position1), # Position value
    #
    360*math.pi*odrv1.axis0.pos_vel_mapper.vel/180, # turns/s to rad/s
    #
    math.sqrt(odrv1.axis0.motor.foc.Iq_measured**2 + odrv1.axis0.motor.foc.Id_measured**2),  # Current value
    #
    -360*(odrv2.axis0.pos_vel_mapper.pos_rel - initial_position2) , # Position value
 ])


# Set up each ODrive
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
odrv0.axis0.controller.config.vel_ramp_rate = 3
odrv0.axis0.controller.config.input_mode = InputMode.TUNING

odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv1.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
odrv1.axis0.controller.config.vel_ramp_rate = 10
odrv1.axis0.controller.config.input_mode = InputMode.VEL_RAMP
odrv1.axis0.controller.input_vel = 0

start_time = time.time()  # Get the current time
    
# try:
#     while True:
#         # Calculate the sin of the elapsed time
#         elapsed_time = time.time() - start_time
#         vel = 5*math.sin(elapsed_time)

#         # Set input velocity
#         odrv0.axis0.controller.input_vel = vel

#         time.sleep(0.1)  # Adjust as needed

try:
    while True:
        # Calculate the elapsed time
        elapsed_time = time.time() - start_time

        # # Generate pulse input
        # if elapsed_time < 0.1:  # Set velocity for the first second
        #     vel = 0
        # else:
        #     vel = 5

        odrv0.axis0.controller.input_pos = math.sin(elapsed_time)+initial_position0
        odrv1.axis0.controller.input_vel = 10*math.sin(elapsed_time)

        # Add the data to the lists for odrv0
        current_data_maxon.append( math.sqrt(odrv1.axis0.motor.foc.Iq_measured**2 + odrv1.axis0.motor.foc.Id_measured**2))
        vel_data_maxon.append(360*math.pi*odrv1.axis0.pos_vel_mapper.vel/180)
        position_data_maxon.append(360*(odrv1.axis0.pos_vel_mapper.pos_rel-initial_position1))

        # Add the data to the lists for odrv1
        current_data_r100.append( math.sqrt(odrv0.axis0.motor.foc.Iq_measured**2 + odrv0.axis0.motor.foc.Id_measured**2))
        vel_data_r100.append(360*math.pi*odrv0.axis0.pos_vel_mapper.vel/180)
        position_data_r100.append(360*(odrv0.axis0.pos_vel_mapper.pos_rel-initial_position0))

        time_data.append(elapsed_time)
        output_pos_data.append(360*math.pi*odrv2.axis0.pos_vel_mapper.vel/180)
        output_vel_data.append(360*(odrv2.axis0.pos_vel_mapper.pos_rel-initial_position2))

                
except KeyboardInterrupt:
    with open('csv/data.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time','current_r100', 'Velocity_r100', 'Position_r100','current_maxon', 'Velocity_maxon', 'Position_maxon','output_pos','output_vel'])
        writer.writerows(zip(time_data, current_data_r100, vel_data_r100, position_data_r100,current_data_maxon, vel_data_maxon, position_data_maxon,output_pos_data,output_vel_data))

    # Set velocity to 0 if the program is interrupted
    odrv0.axis0.controller.input_vel = 0
    odrv1.axis0.controller.input_vel = 0

