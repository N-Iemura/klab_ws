import ezc3d
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Load C3D data
c3d = ezc3d.c3d('from_c3d/49_04.c3d')
points = c3d['data']['points']

# Check the shape of the points array
print("Shape of points array:", points.shape)

# Transpose the points array if necessary
if points.shape[0] != 3:
    points = points.transpose(1, 0, 2)

# Extract the number of frames and points
num_frames = points.shape[2]
num_points = points.shape[1]

# Filter out NaN and Inf values
valid_points = points[:, :, np.isfinite(points).all(axis=0)]

# Check the shape of the valid_points array
print("Shape of valid_points array:", valid_points.shape)

# Create a figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Initialize the scatter plotby
scat = ax.scatter(valid_points[0, :, 0], valid_points[1, :, 0], valid_points[2, :, 0])

# Set axis limits
ax.set_xlim(np.min(valid_points[0, :, :]), np.max(valid_points[0, :, :]))
ax.set_ylim(np.min(valid_points[1, :, :]), np.max(valid_points[1, :, :]))
ax.set_zlim(np.min(valid_points[2, :, :]), np.max(valid_points[2, :, :]))

# Update function for animation
def update(frame):
    scat._offsets3d = (valid_points[0, :, frame], valid_points[1, :, frame], valid_points[2, :, frame])
    return scat,

# Create animation
ani = FuncAnimation(fig, update, frames=num_frames, interval=50, blit=False)

# Show the plot
plt.show()