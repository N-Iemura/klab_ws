import odrive
from odrive.enums import *
import time
import math
import matplotlib.pyplot as plt
import numpy as np
import csv
import threading

TORQUE_CONTROL= 1
VELOCITY_CONTROL = 2
MOTOR_CALIBRATION = 4
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

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

start_time = time.time() 
elapsed_time = time.time() - start_time
# Define a function to control the motor
def control_motor(odrv0, odrv1, initial_position0, start_time):
    while True:
        # Calculate the elapsed time
        elapsed_time = time.time() - start_time

        # Generate pulse input
        if elapsed_time < 1:  # Set velocity for the first second
            odrv0.axis0.controller.input_pos = initial_position0
            odrv1.axis0.controller.input_vel = 0
        elif 1 <= elapsed_time < 1.5: 
            odrv0.axis0.controller.input_pos = -2.0+initial_position0
            odrv1.axis0.controller.input_vel = 0
        else:
            odrv0.axis0.controller.input_pos = -2.0 +initial_position0
            odrv1.axis0.controller.input_vel = 0

        time.sleep(0.01)  # Sleep for 0.01 seconds

# Define a function to collect data
def collect_data(odrv0, odrv1,start_time, initial_position0,initial_position1,initial_position2):
    while True:
        
        elapsed_time = time.time() - start_time
        current_data_maxon.append( math.sqrt(odrv1.axis0.motor.foc.Iq_measured**2 + odrv1.axis0.motor.foc.Id_measured**2))
        vel_data_maxon.append(360*math.pi*odrv1.axis0.pos_vel_mapper.vel/180)
        position_data_maxon.append(360*(odrv1.axis0.pos_vel_mapper.pos_rel-initial_position1))

        # Add the data to the lists for odrv1
        current_data_r100.append( math.sqrt(odrv0.axis0.motor.foc.Iq_measured**2 + odrv0.axis0.motor.foc.Id_measured**2))
        vel_data_r100.append(360*math.pi*odrv0.axis0.pos_vel_mapper.vel/180)
        position_data_r100.append(360*(odrv0.axis0.pos_vel_mapper.pos_rel-initial_position0))

        time_data.append(elapsed_time)
        output_vel_data.append(-360*math.pi*odrv2.axis0.pos_vel_mapper.vel/180)
        output_pos_data.append(-360*(odrv2.axis0.pos_vel_mapper.pos_rel-initial_position2))
        
odrive.utils.start_liveplotter(lambda: [
    #1
    # 360*(odrv0.axis0.pos_vel_mapper.pos_rel - initial_position0) , # Position value
    # #2
    360*math.pi*odrv0.axis0.pos_vel_mapper.vel/180, # turns/s to rad/s
    #3
    # math.sqrt(odrv0.axis0.motor.foc.Iq_measured**2 + odrv0.axis0.motor.foc.Id_measured**2),  # Current value
        #1
    # 360*(odrv1.axis0.pos_vel_mapper.pos_rel - initial_position1), # Position value
    #2
    60*math.pi*odrv1.axis0.pos_vel_mapper.vel/180, # turns/s to rad/s
    #3
#     math.sqrt(odrv1.axis0.motor.foc.Iq_measured**2 + odrv1.axis0.motor.foc.Id_measured**2),  # Current value
    # -360*(odrv2.axis0.pos_vel_mapper.pos_rel - initial_position2) , # Position value
    -360*math.pi*odrv2.axis0.pos_vel_mapper.vel/180,
  ])

# Get the start time
start_time = time.time()

# Create threads for motor control and data collection
control_thread = threading.Thread(target=control_motor, args=(odrv0, odrv1, initial_position0, start_time))
collect_thread = threading.Thread(target=collect_data, args=(odrv0, odrv1,start_time, initial_position0,initial_position1,initial_position2))

# Start the threads
control_thread.start()
collect_thread.start()

# Wait for both threads to finish
control_thread.join()
collect_thread.join()