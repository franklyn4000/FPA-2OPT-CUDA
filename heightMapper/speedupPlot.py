import pandas as pd
import matplotlib.pyplot as plt

column = 0

data1 = pd.read_csv('line_averages_gracy.txt', header=None, sep=' ')
runtimes1 = data1.iloc[:, column]
single_threaded_runtime1 = runtimes1[0]
speedup1 = single_threaded_runtime1 / runtimes1

data2 = pd.read_csv('line_averages_laptop.txt', header=None, sep=' ')
runtimes2 = data2.iloc[:, column]
single_threaded_runtime2 = runtimes2[0]
speedup2 = single_threaded_runtime2 / runtimes2

thread_counts1 = [1, 2, 4, 8, 16, 32, 40, 48, 56, 64, 72]
thread_counts2 = [1, 2, 4, 6, 14]

plt.figure(figsize=(10, 8))

plt.plot(thread_counts1, speedup1, marker='o', label='File 1')
#plt.plot(thread_counts2, speedup2, marker='o', label='File 2')

max_thread_count = max(thread_counts1)
plt.plot([1, max_thread_count], [1, max_thread_count], 'r--')

plt.xlabel('Threads', fontsize=26)
plt.ylabel('Speedup', fontsize=26)
#plt.title('CPU Full Algorithm Speedup')
plt.grid(True)
plt.margins(0)
plt.tight_layout()
plt.show()
