import time
import math
import matplotlib.pyplot as plt
import numpy as np
import csv
from datetime import datetime
from collections import deque

start_time = time.time()
prev_time = time.time()
time_diff = time.time() - prev_time
elapsed_time = time.time() - start_time 

l1 = 0.45
l2 = 0.5
dt = 0.01
m = 0.45

desired_pos0 = 0.5
desired_pos1 = 0.7

t_list = np.linspace(0, 0.5, num=100)
v_list = np.linspace(0, 0.5, num=100)
x = 0
v = 0.1

fx_list = []
torque0_list = []
torque1_list = []

# Initialize variables
fx = 0
new_torque0 = 0
new_torque1 = 0

# Simulation loop
for t in t_list:
    if 0 < t <= 0.05:
        fx = 0
        new_torque0 = 0
        new_torque1 = 0
    elif 0.05 < t < 0.06:
        fx = m * v / dt
        desired_pos0 = np.arcsin((x + l2 * np.sin(desired_pos1)) / l1)
        desired_pos1 = np.arccos((0.7 - l1 * np.cos(desired_pos0)) / l2)
        new_torque0 = l1 * np.cos(desired_pos0) * fx
        new_torque1 = -(0.7 + l1 * np.cos(desired_pos0)) / l2 * fx
    elif t >= 0.06:
        fx = 0
        new_torque0 = 0
        new_torque1 = 0

    # Append values to lists
    fx_list.append(fx)
    torque0_list.append(new_torque0)
    torque1_list.append(new_torque1)

# Plotting
plt.figure(figsize=(12, 6))

# Plot fx
plt.subplot(2, 1, 1)
plt.plot(t_list, fx_list, label='fx')
plt.xlabel('Time')
plt.ylabel('fx')
plt.legend()

# Plot new_torque0 and new_torque1 on the same graph
plt.subplot(2, 1, 2)
plt.plot(t_list, torque0_list, label='new_torque0', color='orange')
plt.plot(t_list, torque1_list, label='new_torque1', color='green')
plt.xlabel('Time')
plt.ylabel('Torque')
plt.legend()

plt.tight_layout()
plt.show()