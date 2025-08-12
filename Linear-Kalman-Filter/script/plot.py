import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D

# Load the data
data = pd.read_csv("output/kf_output.csv")
idata = pd.read_csv("data/target_tracking.csv")

# Extract data columns
ox = np.array(idata.x)
oy = np.array(idata.y)
oz = np.array(idata.z)

x = np.array(data.x)
y = np.array(data.y)
z = np.array(data.z)
vx = np.array(data.vx)
vy = np.array(data.vy)
vz = np.array(data.vz)

# Plot setup
area = 250
show_animation = True

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the ground truth trajectory
ax.plot3D(ox, oy, oz, label="Ground Truth", alpha=0.5, color='red')

# Animation loop
if show_animation:
    est_path, = ax.plot3D([], [], [], color='blue', label="Estimated Path", linewidth=2)

    ax.set_xlim(min(ox) - area, max(ox) + area)
    ax.set_ylim(min(oy) - area, max(oy) + area)
    ax.set_zlim(min(oz) - area, max(oz) + area)

    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_zlabel("Z Position")
    ax.legend()
    ax.grid(True)

    for i in range(len(x)):
        est_path.set_data(x[:i+1], y[:i+1])
        est_path.set_3d_properties(z[:i+1])
        plt.pause(0.05)

print("Animation complete.")
plt.show()
