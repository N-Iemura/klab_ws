import matplotlib.pyplot as plt
import numpy as np
import matplotlib

# Define the function
def increasing_function(t):
    return np.exp(-t)


# Generate time values
t = np.linspace(0, 10, 100)

# Calculate function values
y = increasing_function(t)

matplotlib.rcParams['font.family'] = 'Times New Roman'
matplotlib.rcParams['font.size'] = 12


matplotlib.rcParams['axes.xmargin'] = 0
plt.tick_params(direction='in')
# Plot the function
plt.plot(t, y)
plt.xlabel('Time')
plt.ylabel('Reduction ratio')
plt.show()
