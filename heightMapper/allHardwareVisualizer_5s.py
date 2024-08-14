import pandas as pd
import matplotlib.pyplot as plt

# Define file names
files = ["../plotdata/fitevoomp5.dat", "../plotdata/fitevocuda5.dat", "../plotdata/fitevoomp5.dat",
         "../plotdata/fitevocuda5.dat", "../plotdata/fitevoomp5.dat", "../plotdata/fitevocuda5.dat"]

# Empty list to store maximum values
max_values = []

# Iterate over each file
for file in files:
    # Read the file into pandas DataFrame considering space-seprated values
    df = pd.read_csv(file, names=["col1", "col2"],  sep='\s+')

    # Find the maximum value in the second column and append to the list
    max_values.append(df["col2"].max())

# Custom labels
labels = ['GH200 OMP', 'GH200 CUDA', 'Laptop OMP', 'Laptop CUDA', 'Jetson OMP', 'Jetson CUDA']

colors = ['#76B900' if i % 2 == 1 else 'red' for i in range(len(files))]

# Plotting
plt.bar(range(len(files)), max_values, color=colors)
plt.xlabel('Files')
plt.ylabel('Max value')
plt.xticks(range(len(files)), labels, rotation='vertical')  # Replace 'files' with 'labels'
plt.ylim(1.3, 1.5)
plt.title('Maximum values in 2nd column of each file')


y = 1.2
while y <= 1.5:
    plt.axhline(y, color='gray', linewidth=0.3)
    y += 0.025

# Show the plot
plt.tight_layout()
plt.show()
