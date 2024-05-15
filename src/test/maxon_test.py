import odrive
from odrive.enums import *
import time
import math
import matplotlib.pyplot as plt
import numpy as np


MOTOR_CALIBRATION = 4
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
# VELOCITY_CONTROL = 2

# Connect to the ODrive
odrv0 = odrive.find_any(serial_number='385E344A3539')

odrv0.axis0.requested_state = MOTOR_CALIBRATION
time.sleep(5) 

odrive.utils.start_liveplotter(lambda: [
    #1
    odrv0.axis0.pos_vel_mapper.pos_rel, # Position value
    #2
    odrv0.axis0.pos_vel_mapper.vel, # turns/s to rpm 
    #3
    odrv0.axis0.motor.foc.Iq_measured,  # Current value
 ])

odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# Set control mode to velocity control
odrv0.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL

odrv0.axis0.controller.config.vel_ramp_rate = 5

odrv0.axis0.controller.config.input_mode = InputMode.VEL_RAMP

odrv0.axis0.controller.input_vel = 0

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
        if elapsed_time % 2 < 1:  # Change velocity every second
            vel = 7
        else:
            vel = 0

        # Set input velocity
        odrv0.axis0.controller.input_vel = vel
        
except KeyboardInterrupt:
    # Set velocity to 0 if the program is interrupted
    odrv0.axis0.controller.input_vel = 0