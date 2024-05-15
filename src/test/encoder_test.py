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

# odrv2 = odrive.find_any(serial_number='3849346f3539')

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
    # #2
    360*odrv1.axis0.pos_vel_mapper.pos_rel,
    # #3
    360*odrv2.axis0.pos_vel_mapper.pos_rel
 ])

# Set up each ODrive
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
odrv0.axis0.controller.config.vel_ramp_rate = 1
odrv0.axis0.controller.config.input_mode = InputMode.TUNING
odrv0.axis0.controller.input_pos = 0

odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv1.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
odrv1.axis0.controller.config.vel_ramp_rate = 1
odrv1.axis0.controller.config.input_mode = InputMode.TUNING
odrv1.axis0.controller.input_pos = 0


# Set up each ODrive
# odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
# odrv0.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
# odrv0.axis0.controller.config.vel_ramp_rate = 5
# odrv0.axis0.controller.config.input_mode = InputMode.VEL_RAMP
# odrv0.axis0.controller.input_vel = 0

# odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
# odrv1.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
# odrv1.axis0.controller.config.vel_ramp_rate = 5
# odrv1.axis0.controller.config.input_mode = InputMode.VEL_RAMP
# odrv1.axis0.controller.input_vel = 0

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
            vel = 0

        # Set input velocity for each ODrive
        # odrv0.axis0.controller.input_vel = 0
        # odrv1.axis0.controller.input_vel = -vel

         # Get the torque and position from odrv0
        torque_odrv0 = odrv0.axis0.motor.foc.Iq_measured
        vel_odrv0 = odrv0.axis0.pos_vel_mapper.vel
        position_odrv0 = odrv0.axis0.pos_vel_mapper.pos_rel
        # Get the torque and position from odrv1
        torque_odrv1 = odrv1.axis0.motor.foc.Iq_measured
        vel_odrv1 = odrv1.axis0.pos_vel_mapper.vel
        position_odrv1 = odrv1.axis0.pos_vel_mapper.pos_rel

        # Add the data to the lists for odrv0
        torque_data_maxon.append(torque_odrv1)
        vel_data_maxon.append(vel_odrv1)
        position_data_maxon.append(position_odrv1)

        # Add the data to the lists for odrv1
        torque_data_r100.append(torque_odrv0)
        vel_data_r100.append(vel_odrv0)
        position_data_r100.append(position_odrv0)

           
except KeyboardInterrupt:
    # Set velocity to 0 if the program is interrupted
    odrv0.axis0.controller.input_vel = 0
    odrv1.axis0.controller.input_vel = 0

