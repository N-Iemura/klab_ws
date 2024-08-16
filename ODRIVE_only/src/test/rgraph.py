import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib
import numpy as np
import scipy
from scipy import signal

# Set the global font to be Times New Roman, size 10 (or any other size you want)
matplotlib.rcParams['font.family'] = 'Times New Roman'
matplotlib.rcParams['font.size'] = 16

matplotlib.rcParams['axes.xmargin'] = 0
matplotlib.rcParams['axes.ymargin'] = 0

# Read the CSV data
# data = pd.read_csv('/home/naga/klab_ws/src/test/csv/l1_-1.3_-10_1.csv',encoding = 'UTF8')
data = pd.read_csv('/home/naga/klab_ws/src/test/s1/s1_1_2-12-6.csv',encoding = 'UTF8')
t = data['time']
# plt.plot(t.to_numpy(),data['Velocity_r100'].to_numpy(),label='R100')
# plt.plot(t.to_numpy(),data['Velocity_maxon'].to_numpy(),label='R100')
# data['Velocity_r100'].plot(x=data['time'])
# data['Velocity_maxon'].plot(x=data['time'])
# (-data['output_vel']).plot(x=data['time'])

# Create a subplot with 3 rows and 1 column
fig, ax = plt.subplots(figsize=(8.0, 4.0))

# Generate time data starting from 0.25s
t_alternating = np.arange(0.25, t.max(), 0.5)

# Generate alternating +5 and -5 data
alternating_data = np.tile([6, -6], len(t_alternating) // 2)

alternating_data2 = np.tile([5, -5], len(t_alternating) // 2)

alternating_data2[:10] = np.tile([2, -2], len(t_alternating[:10]) // 2)  # 0-5 seconds
alternating_data2[10:20] = np.tile([12, -12], len(t_alternating[10:20]) // 2)  # 5-10 seconds
alternating_data2[20:] = np.tile([6, -6], len(t_alternating[20:]) // 2)  # 10-15 seconds

# If t_alternating is longer
if len(t_alternating) > len(alternating_data):
    t_alternating = t_alternating[:len(alternating_data)]

# If alternating_data is longer
elif len(alternating_data) > len(t_alternating):
    alternating_data = alternating_data[:len(t_alternating)]

# If t_alternating is longer
if len(t_alternating) > len(alternating_data2):
    t_alternating = t_alternating[:len(alternating_data2)]

# If alternating_data is longer
elif len(alternating_data2) > len(t_alternating):
    alternating_data2 = alternating_data2[:len(t_alternating)]


reduction_data=7/(1+alternating_data2/alternating_data)
reduction_data_f=np.abs(data['Velocity_r100']/(-data['output_vel']))

t_alternating_s = np.arange(0, 15, 0.05)

# Generate square wave data
square_wave_data = np.repeat([5, -5], 10)

t1 = np.linspace(0, 15, 50, endpoint=False)
# ax.plot(t1, signal.square(2 * np.pi * 5 * t1))



# Plot the alternating data
# ax.plot(t_alternating, alternating_data, label='motor1', color='red')

# ax.plot(t_alternating, alternating_data, label='motor1', color='red')

# ax.plot(t_alternating, alternating_data2, label='motor2', color='blue')

# ax2 = ax.twinx()

# ax2.plot(t_alternating, reduction_data, label='reduction ratio', color='orange')

# ax.plot(t.to_numpy(), reduction_data_f.to_numpy(), label='motor2', color='yellow')

ax.plot(t_alternating, alternating_data/reduction_data, label='output_ref', color=(0.5, 0.25, 0.5))

# Plot 'Velocity_maxon' on the second subplot
# Plot 'Velocity_r100'
# ax.plot(t.to_numpy(), data['Velocity_r100'].to_numpy(), label='motor1', color='red')

# Plot 'Velocity_maxon'
# ax.plot(t.to_numpy(), (-data['Velocity_maxon']/6).to_numpy(), label='motor2', color='blue')

# Plot 'output_vel'
ax.plot(t.to_numpy(), (-data['output_vel']).to_numpy(), label='output_res', color=(0.1, 0.8, 0.1))

ax.axvline(x=5, color='black', linestyle=':')
ax.axvline(x=10, color='black', linestyle=':')

# Set labels and ticks
ax.set_xlabel('Time [s]')  # Set the x-axis label to 'Time'
ax.set_ylabel('Velocity [rad/s]')  # Set the y-axis label to 'Velocity'
ax.tick_params(direction='in')  
ax.set_ylim([-3, 3]) 
ax.set_xlim([0, 15]) 
# ax2.set_ylim([0, 6]) 
# ax2.set_ylabel('Reduction ratio')  # Set the y-axis label to 'Velocity'
# ax2.tick_params(direction='in') 

# Set legend
lines, labels = ax.get_legend_handles_labels()
ax.legend(lines, labels, loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=3, frameon=False)
# ax2.legend(loc='upper center', bbox_to_anchor=(0.75, 1.15), ncol=3, frameon=False)

plt.show()

plt.show()