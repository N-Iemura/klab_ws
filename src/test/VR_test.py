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

# odrv0.axis0.requested_state = MOTOR_CALIBRATION
# time.sleep(7) 
torque_data_maxon = []
vel_data_maxon = []
position_data_maxon = []
torque_data_r100 = []
vel_data_r100 = []
position_data_r100 = []

odrive.utils.start_liveplotter(lambda: [
    #1
    360*odrv0.axis0.pos_vel_mapper.pos_rel, # Position value
    #2
    360*math.pi*odrv0.axis0.pos_vel_mapper.vel/180, # turns/s to rad/s
    #3
    odrv0.axis0.motor.foc.Iq_measured,  # Current value
        #1
    360*odrv1.axis0.pos_vel_mapper.pos_rel, # Position value
    #2
    360*math.pi*odrv1.axis0.pos_vel_mapper.vel/180, # turns/s to rpm 
    #3
    odrv1.axis0.motor.foc.Iq_measured,  # Current value
 ])


# Set up each ODrive
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
odrv0.axis0.controller.config.vel_ramp_rate = 5
odrv0.axis0.controller.config.input_mode = InputMode.VEL_RAMP
odrv0.axis0.controller.input_vel = 0

odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv1.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
odrv1.axis0.controller.config.vel_ramp_rate = 5
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

        # Generate pulse input
        if elapsed_time < 1:  # Set velocity for the first second
            vel = 0
        else:
            vel = 5

        # Set input velocity for each ODrive
        odrv0.axis0.controller.input_vel = 5
        odrv1.axis0.controller.input_vel = -vel

         # Get the torque and position from odrv0
        torque_odrv0 = odrv0.axis0.motor.foc.Iq_measured
        vel_odrv0 = 360*math.pi*odrv0.axis0.pos_vel_mapper.vel/180
        position_odrv0 = 360*odrv0.axis0.pos_vel_mapper.pos_rel
        # Get the torque and position from odrv1
        torque_odrv1 = odrv1.axis0.motor.foc.Iq_measured
        vel_odrv1 = 360*math.pi*odrv1.axis0.pos_vel_mapper.vel/180
        position_odrv1 = 360*odrv1.axis0.pos_vel_mapper.pos_rel

        # Add the data to the lists for odrv0
        torque_data_maxon.append(torque_odrv1)
        vel_data_maxon.append(vel_odrv1)
        position_data_maxon.append(position_odrv1)

        # Add the data to the lists for odrv1
        torque_data_r100.append(torque_odrv0)
        vel_data_r100.append(vel_odrv0)
        position_data_r100.append(position_odrv0)

                
except KeyboardInterrupt:
    with open('csv/data.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Torque_maxon', 'Velocity_maxon', 'Position_maxon', 'Torque_r100', 'Velocity_r100', 'Position_r100'])
        writer.writerows(zip(torque_data_maxon, vel_data_maxon, position_data_maxon, torque_data_r100, vel_data_r100, position_data_r100))

    # Set velocity to 0 if the program is interrupted
    odrv0.axis0.controller.input_vel = 0
    odrv1.axis0.controller.input_vel = 0

