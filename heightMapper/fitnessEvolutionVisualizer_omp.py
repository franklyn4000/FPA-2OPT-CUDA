

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read data from the first and second file
df1 = pd.read_csv("../plotdata/fitevoomp.dat", sep='\s+', names=["x", "y"])
df2 = pd.read_csv("../plotdata/fitevoomp5.dat", sep='\s+', names=["x", "y"])

# Create a log-scaled plot and plot both datasets
plt.figure(figsize=(8, 6))
plt.semilogx(df1["x"], df1["y"], label='1 s runtime')
plt.semilogx(df2["x"], df2["y"], label='5 s runtime')

# Scaling the y-axis
plt.ylim([1.2, 1.5])

# Create legend
plt.legend()

# Get current axis
ax = plt.gca()

# Merge x values from both datasets for the x-ticks
x_ticks = np.concatenate((df1["x"].values, df2["x"].values))
ax.set_xticks(x_ticks)
ax.set_ylabel('fitness', color='black', fontsize=22)

# Set x-tick labels as the string representation of the tick values
ax.set_xticklabels(x_ticks.astype(str), rotation=90)

plt.xlabel('threads', color='black', fontsize=22)

y = 1.2
while y <= 1.5:
    plt.axhline(y, color='gray', linewidth=0.3)
    y += 0.05

# Show the plot
plt.tight_layout()
plt.show()