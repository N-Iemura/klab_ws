import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib
import numpy as np

# Set the global font to be Times New Roman, size 10 (or any other size you want)
matplotlib.rcParams['font.family'] = 'Times New Roman'
matplotlib.rcParams['font.size'] = 15

matplotlib.rcParams['axes.xmargin'] = 0
matplotlib.rcParams['axes.ymargin'] = 0

# Read the CSV data
# data = pd.read_csv('/home/naga/klab_ws/src/test/csv/l1_-1.3_-10_1.csv',encoding = 'UTF8')

data = pd.read_csv('/home/naga/klab_ws/src/test/a.csv',encoding = 'UTF8')
t = data['omega']
# plt.plot(t.to_numpy(),data['Velocity_r100'].to_numpy(),label='R100')
# plt.plot(t.to_numpy(),data['Velocity_maxon'].to_numpy(),label='R100')
# data['Velocity_r100'].plot(x=data['time'])
# data['Velocity_maxon'].plot(x=data['time'])
# (-data['output_vel']).plot(x=data['time'])

# Create a subplot with 3 rows and 1 column
fig, ax = plt.subplots(figsize=(8.0, 4.0))



# Plot 'Velocity_maxon' on the second subplot


# Plot 'Velocity_maxon'
ax.plot(t.to_numpy(), (5*data['m1']).to_numpy(), '-o',  label='m1', color='blue')

# Plot 'output_vel'
ax.plot(t.to_numpy(), (5*data['m2']).to_numpy(), '-o',  label='m2', color='red')

ax.plot(t.to_numpy(), (5*data['m3']).to_numpy(),  '-o', label='m3', color=(0.1, 0.8, 0.1))

# Set labels and ticks
ax.set_xlabel('Velocity [rad/s]')  # Set the x-axis label to 'Time'
# ax.set_xlabel('Reduction ratio')  # Set the x-axis label to 'Time'
ax.set_ylabel('hhhh [cm]')  # Set the y-axis label to 'Velocity'
ax.tick_params(direction='in')  
ax.set_ylim([30, 70]) 
ax.set_xlim([-15, 45]) 
# ax.xaxis.set_ticks(np.arange(0, 21, 2))
# Assuming 'x' and 'y' are the coordinates of the points



# Set legend
lines, labels = ax.get_legend_handles_labels()
ax.legend(lines, labels, loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=3, frameon=False)

plt.show()

