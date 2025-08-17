import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
import sys

cols = [
    'sec', 'nsec', 'frame_id', 
    'x', 'y', 'z', 
    'qx', 'qy', 'qz', 'qw'
]
kf = pd.read_csv('kf_pose.csv', names=cols)
gt = pd.read_csv('true_pose.csv', names=cols)


kf['t'] = kf['sec'] + kf['nsec'] * 1e-9
gt['t'] = gt['sec'] + gt['nsec'] * 1e-9

# Match timestamps
t_start = max(kf['t'].iloc[0], gt['t'].iloc[0])
t_end   = min(kf['t'].iloc[-1], gt['t'].iloc[-1])
kf = kf[(kf['t'] >= t_start) & (kf['t'] <= t_end)].reset_index(drop=True)
gt = gt[(gt['t'] >= t_start) & (gt['t'] <= t_end)].reset_index(drop=True)


# Plot the 2D path:
plt.figure(figsize=(7,5))
plt.plot(kf['x'], kf['y'], label='EKF trajectory')
plt.plot(gt['x'], gt['y'], label='Ground truth')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('EKF Estimation vs Ground Truth')
plt.legend()
plt.tight_layout()
plt.savefig('EKF_vs_GT_trajectory_Carlike_robot.png', dpi=300)
plt.show()

# --- Animation Setup ---
gt_xi = np.interp(kf['t'], gt['t'], gt['x'])
gt_yi = np.interp(kf['t'], gt['t'], gt['y'])

def quat_to_yaw(qx,qy,qz,qw):
    return np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
kf_theta = quat_to_yaw(kf['qx'], kf['qy'], kf['qz'], kf['qw'])
gt_theta = quat_to_yaw(gt['qx'], gt['qy'], gt['qz'], gt['qw'])
gt_theta_i = np.interp(kf['t'], gt['t'], gt_theta)

pos_err = np.hypot(kf['x'] - gt_xi, kf['y'] - gt_yi)
rms_pos  = np.sqrt(np.mean(pos_err**2))

heading_err = np.unwrap(kf_theta - gt_theta_i)   # unwrap to avoid 2π jumps
rms_theta = np.sqrt(np.mean(heading_err**2))

print(f"RMS position error  : {rms_pos:.4f} m")
print(f"RMS heading  error  : {rms_theta:.4f} rad  ({np.degrees(rms_theta):.2f}°)")

sys.exit()
min_x = min(kf['x'].min(), gt_xi.min()) - 1
max_x = max(kf['x'].max(), gt_xi.max()) + 1
min_y = min(kf['y'].min(), gt_yi.min()) - 1
max_y = max(kf['y'].max(), gt_yi.max()) + 1

fig, ax = plt.subplots(figsize=(7,5))
ax.set_xlim(min_x, max_x)
ax.set_ylim(min_y, max_y)
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_title('EKF vs Ground Truth Animation')
line_kf, = ax.plot([], [], '-', lw=3, label='EKF')
line_gt, = ax.plot([], [], '--', lw=3, label='GT')
ax.legend()

def update(frame):
    line_kf.set_data(kf['x'][:frame], kf['y'][:frame])
    line_gt.set_data(gt_xi[:frame], gt_yi[:frame])
    return line_kf, line_gt

ani = FuncAnimation(fig, update, frames=len(kf), interval=20, blit=True)
ani.save('EKF_vs_GT_animation.gif', writer=PillowWriter(fps=40))