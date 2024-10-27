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
# maxon
odrv0 = odrive.find_any(serial_number='385B34743539')
#R100
odrv1 = odrive.find_any(serial_number='385E344A3539')

odrv0.axis0.requested_state = MOTOR_CALIBRATION
odrv1.axis0.requested_state = MOTOR_CALIBRATION