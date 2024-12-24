import odrive
from odrive.enums import *
import time
import math
import matplotlib.pyplot as plt
import numpy as np

TORQUE_CONTROL= 1
MOTOR_CALIBRATION = 4
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
VELOCITY_CONTROL = 2

# Find two ODrives
odrv0 = odrive.find_any(serial_number='385B34743539')
odrv1 = odrive.find_any(serial_number='385E344A3539')

# odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
# odrv0.axis0.requested_state = AxisState.MOTOR_CALIBRATION
# odrv1.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
# odrv1.axis0.requested_state = AxisState.MOTOR_CALIBRATION
odrv0.axis0.requested_state = AxisState.IDLE
odrv1.axis0.requested_state = AxisState.IDLE

# odrv0.axis0.requested_state = MOTOR_CALIBRATION
# odrv1.axis0.requested_state = MOTOR_CALIBRATION
