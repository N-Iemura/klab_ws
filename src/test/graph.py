import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib
import numpy as np

# Set the global font to be Times New Roman, size 10 (or any other size you want)
matplotlib.rcParams['font.family'] = 'Times New Roman'
matplotlib.rcParams['font.size'] = 12

matplotlib.rcParams['axes.xmargin'] = 0
matplotlib.rcParams['axes.ymargin'] = 0

# Read the CSV data
# data = pd.read_csv('/home/naga/klab_ws/src/test/csv/l1_-1.3_-10_1.csv',encoding = 'UTF8')
data = pd.read_csv('/home/naga/klab_ws/src/test/m2/m2_0_1.csv',encoding = 'UTF8')
t = data['time']
# plt.plot(t.to_numpy(),data['Velocity_r100'].to_numpy(),label='R100')
# plt.plot(t.to_numpy(),data['Velocity_maxon'].to_numpy(),label='R100')
# data['Velocity_r100'].plot(x=data['time'])
# data['Velocity_maxon'].plot(x=data['time'])
# (-data['output_vel']).plot(x=data['time'])

# Create a subplot with 3 rows and 1 column
fig, ax = plt.subplots(figsize=(8.0, 4.0))


# Plot 'Velocity_maxon' on the second subplot
# Plot 'Velocity_r100'
# ax.plot(t.to_numpy(), (7/(1+abs(data['Velocity_maxon'])/abs(data['Velocity_r100']))).to_numpy(), label='motor1', color='red')
# ax.plot(t.to_numpy(), (abs(data['Velocity_r100'])/abs(data['output_vel'])).to_numpy(), label='motor2', color='blue')


# # Plot 'Velocity_maxon'
ax.plot(t.to_numpy(), (-data['Velocity_r100']).to_numpy(), label='motor1', color='red')
ax.plot(t.to_numpy(), (data['Velocity_maxon']/6).to_numpy(), label='motor2', color='blue')
ax.plot(t.to_numpy(), (data['output_vel']).to_numpy(), label='motor3', color='green')

# # Plot 'output_vel'
# ax.plot(t.to_numpy(), (-data['output_vel']).to_numpy(), label='output', color=(0.1, 0.8, 0.1))

# Set labels and ticks
ax.set_xlabel('Time [s]')  # Set the x-axis label to 'Time'
ax.set_ylabel('Velocity [rad/s]')  # Set the y-axis label to 'Velocity'
ax.tick_params(direction='in')  


# Set legend
lines, labels = ax.get_legend_handles_labels()
ax.legend(lines, labels, loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=3, frameon=False)

plt.show()