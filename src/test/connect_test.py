import odrive
from odrive.enums import *
import time

# Connect to the ODrive
odrv0 = odrive.find_any()

# Ensure motor is calibrated
if not odrv0.axis0.motor.config.pre_calibrated:
    print("Calibrating motor...")
    odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    while odrv0.axis0.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

# Ensure the encoder is ready
if not odrv0.axis0.encoder.is_ready:
    print("Encoder is not ready")
    # Handle the situation here

# Set control mode to velocity control
odrv0.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL

# Set velocity limit
odrv0.axis0.controller.config.vel_limit = 100000

# Set input velocity
odrv0.axis0.controller.input_vel = 1

# Set the motor to closed loop control
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL