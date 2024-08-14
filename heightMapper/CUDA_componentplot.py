import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# File paths
files = ['line_averages_cuda_gracy.txt', 'line_averages_cuda_laptop.txt']

# Empty list to store averages
averages = []

# Read from each file, calculate percentages, and add the averages to the list
for file in files:
    df = pd.read_csv(file, header=None, sep='\s+')
    df_percent = df.iloc[:, 1:6].div(df.iloc[:, 0], axis=0)
    averages.append(df_percent.mean())

# Prepare for the multi-bar plot
labels = ['Data transfer', 'Pollination', 'Smoothing', 'Fitnesses', '2-OPT']
x = np.arange(len(labels))
width = 0.20

# Create subplots
fig, ax = plt.subplots()

# Add bars for each file data
rects1 = ax.bar(x - width * 1.5, averages[0] * 100, width, label='Grace Hopper')
rects2 = ax.bar(x - width / 2, averages[1] * 100, width, label='Laptop')
#rects3 = ax.bar(x + width / 2, averages[2] * 100, width, label='File 3')

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Percentage')
ax.set_title('GPU Runtime percentage of each algorithm component')
ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.legend()

fig.tight_layout()

plt.show()