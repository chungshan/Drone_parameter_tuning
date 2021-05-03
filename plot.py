import numpy as np
import os
import matplotlib.pyplot as plt
log_dir = "pose"

log_files = sorted(os.listdir(log_dir))
fig, ax = plt.subplots()
fig.suptitle("Height")
ax.axhline(y=1, color = 'r', linestyle='--')
for log_ in log_files:
    pose = np.loadtxt(os.path.join(log_dir,log_))
    pose_y = pose[:, 1]
    ax.plot(range(0, len(pose_y)), pose_y, label=log_.split('_')[1])

ax.set_xlim(0, len(pose_y))
ax.legend()

plt.show()