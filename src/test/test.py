import odrive
from odrive.enums import *
import time
import math


MOTOR_CALIBRATION = 4
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
# VELOCITY_CONTROL = 2

# Connect to the ODrive
odrv0 = odrive.find_any()

odrv0.axis0.requested_state = MOTOR_CALIBRATION
time.sleep(5) 

odrive.utils.start_liveplotter(lambda: [
    odrv0.axis0.pos_vel_mapper.vel * 60, # turns/s to rpm 
    odrv0.axis0.motor.foc.Iq_measured,  # Current value
 ])

odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# Set control mode to velocity control
odrv0.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL

odrv0.axis0.controller.config.vel_ramp_rate = 0.5

odrv0.axis0.controller.config.input_mode = InputMode.VEL_RAMP

start_time = time.time()  # Get the current time

try:
    while True:
        # Calculate the sin of the elapsed time
        elapsed_time = time.time() - start_time
        vel = math.sin(elapsed_time)

        # Set input velocity
        odrv0.axis0.controller.input_vel = vel

        time.sleep(0.1)  # Adjust as needed
        
except KeyboardInterrupt:
    # Set velocity to 0 if the program is interrupted
    odrv0.axis0.controller.input_vel = 0