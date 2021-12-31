'''!@example UseCubicSmoothSpline.py
@brief How to use cubic smooth spline.
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from RVBUST.RPS import *
from IPython import embed

np.random.seed(1)
nums = 100
repeat = 5
t = np.linspace(0, np.pi * 2, nums)
x = np.sin(t) * 2
y = np.cos(t) * 2
x += np.random.randn(nums) * 0.1
y += np.random.randn(nums) * 0.1
z = 0.5 * t + np.random.randn(nums) * 0.1

data = np.c_[x, y, z]
weights = np.ones_like(t)
weights[[0, -1]] = 100
spl = CubicSplineRn(data, t, weights, 0.9)
data_f = np.array([spl(s) for s in t])

plt.subplot(211)
plt.plot(data, "r*--", label="origin path", lw=1, ms=2)
plt.plot(data_f, "g^-", label="cubic smooth spline", lw=1, ms=2)
plt.legend()
plt.subplot(212, projection='3d')
plt.plot(*data.T, label="origin path")
plt.plot(*data_f.T, label="cubic smooth spline")
plt.legend()
plt.show()

dof = 3
vel_limits = np.ones(dof)
acc_limits = np.ones(dof)
jerk_limits = np.ones(dof)
path = spl.ToPath()
traj = CreateTrajectory(path, vel_limits, acc_limits,
                        jerk_limits, traj_type=TrajType_Toppra)
DrawTraj(traj)
embed()
