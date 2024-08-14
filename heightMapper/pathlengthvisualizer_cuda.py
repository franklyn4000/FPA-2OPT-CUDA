import matplotlib.pyplot as plt
import numpy as np

# Read data from file and unpack it into three lists
data = np.genfromtxt("../plotdata/pathevo-2.dat")
x_values, y_values1, y_values2 = data[:, 0], data[:, 1], data[:, 2]

fig, ax1 = plt.subplots()

# Plot first line with y-axis on the left
color = 'tab:blue'
ax1.set_ylabel('fitness', color=color, fontsize=22)  # we already handled the x-label with ax1
ax1.plot(x_values, y_values2, color=color)
ax1.tick_params(axis='y', labelcolor=color)
ax1.set_ylim([1.2, 1.4])  # Setting the limit for first y-axis

# instantiate a second axes that shares the same x-axis
ax2 = ax1.twinx()

# Plot second line with y-axis on the right
color = 'tab:red'
ax2.set_ylabel('runtime (s)', color=color, fontsize=22)  # we already handled the x-label with ax1
ax2.plot(x_values, y_values1, color=color)
ax2.tick_params(axis='y', labelcolor=color)
ax2.set_ylim([0, 40])  # Setting the limit for second y-axis

ax1.set_xlabel("Path length", fontsize=22)

plt.margins(0)
plt.tight_layout()
plt.show()
