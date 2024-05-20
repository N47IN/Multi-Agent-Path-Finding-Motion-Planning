import pandas as pd
import matplotlib.pyplot as plt

# Define the file path to your CSV
file_path = '/home/navin/catkin_ws/src/Multi-Agent-Path-Finding-Motion-Planning/Webots_Scripts/rrt_wosmoothed_global.csv'

# Read the CSV file
df = pd.read_csv(file_path)

# Check if the DataFrame has enough rows
if df.shape[0] < 3:
    raise ValueError("The CSV file does not have enough rows.")

# Extract the second and third rows, excluding the first element of each row
row2 = df.iloc[1:, 1]
row3 = df.iloc[1:, 2]
print(row2)
# Plot the data
plt.figure(figsize=(10, 6))
plt.plot(row2, color='orange', label='Speed')
plt.plot(row3, color='b', label='Steering')
plt.xlabel('Time (ms)')
plt.ylabel('Values')
plt.title('Speed and steering plot')
plt.legend()
plt.savefig("realrrt_nosmooth.png")
plt.grid(True)
plt.show()