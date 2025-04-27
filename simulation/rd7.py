import matplotlib.pyplot as plt
import numpy as np

# Torque values (approximate x-axis range)
torque = np.linspace(0, 11, 50)

# Speed (RPM) - Concave-down quadratic approximation, starting from around 3800 RPM and ending around 2300 RPM
# Adjust coefficients to ensure speed is 2300 when torque is 12 and 3800 when torque is 0
# Vertex (axis) is at torque = 15
vertex = 15
a = (3800 - 2300) / ((0 - vertex)**2 - (12 - vertex)**2)
b = -2 * a * vertex
c = 3800

speed = a * torque**2 + b * torque + c

# Plotting RPM vs. Torque for different reduction ratios
plt.figure(figsize=(8, 6))

ratios = np.linspace(7, 33, 500)  # Continuous range of reduction ratios

# Plot all other ratios with a light color
first_fill = True
for ratio in ratios:
    pass
#     if not np.isclose(ratio, 7) and not np.isclose(ratio, 33):
#         reduced_speed = speed / ratio
#         plt.plot(torque * ratio, reduced_speed, color='orange', alpha=0.1)
#         if first_fill:
#             plt.fill_between(torque * ratio, reduced_speed, color='orange', alpha=0.4, label="Reduction ratio: 7-33")
#             first_fill = False
#         else:
#             plt.fill_between(torque * ratio, reduced_speed, color='orange', alpha=0.3)

# # Plot ratio 7 with a distinct color
ratio = 7
reduced_speed = speed / ratio
plt.plot(torque * ratio, reduced_speed, color='blue', alpha=0.1, linewidth=1)
plt.fill_between(torque * ratio, reduced_speed, color='blue', alpha=0.5, label="Reduction ratio: 7")

# ratio = 8
# reduced_speed = speed / ratio
# plt.plot(torque * ratio, reduced_speed, color='green', alpha=0.1, linewidth=1)
# plt.fill_between(torque * ratio, reduced_speed, color='green', alpha=0.5, label="Reduction ratio: 8")

# ratio = 9
# reduced_speed = speed / ratio
# plt.plot(torque * ratio, reduced_speed, color='purple', alpha=0.1, linewidth=1)
# plt.fill_between(torque * ratio, reduced_speed, color='purple', alpha=0.5, label="Reduction ratio: 9")

# Plot ratio 33 with a distinct color
# ratio = 33
# reduced_speed = speed / ratio
# plt.plot(torque * ratio, reduced_speed, color='red', alpha=0.1, linewidth=1)
# plt.fill_between(torque * ratio, reduced_speed, color='red', alpha=0.4, label="Reduction ratio: 33")

# Set font properties for axis labels
font_properties = {'family': 'Times New Roman', 'size': 18}

plt.xlabel('Torque [Nm]', fontdict=font_properties)
plt.ylabel('Speed [rpm]', fontdict=font_properties)
plt.ylim(0, 580)
plt.xlim(0, 380)
plt.xticks(fontsize=16, fontname='Times New Roman')
plt.yticks(fontsize=16, fontname='Times New Roman')
legend = plt.legend(prop=font_properties)
legend.get_frame().set_linewidth(0)

# Save the plot as an SVG file
plt.savefig('plot7.svg', format='svg')

plt.show()