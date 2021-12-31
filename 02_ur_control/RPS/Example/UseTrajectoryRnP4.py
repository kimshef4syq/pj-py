'''!@example UseCreateTrajectoryP4.py
@brief Example of P4 trajectory.
Give a series of waypoints and corresponding timestamps, output the optimal trajectory in comfort with kinematic limits.
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import time
import numpy as np
from IPython import embed

from RVBUST.RPS import *

# Path1: Assume the path is a spiral line in 3d space
s = np.linspace(0, 2*np.pi, 30)
qs = np.array([2.0 * np.sin(s), 2.0 * np.cos(s)]).T
qs = np.c_[qs, s]
dof = qs.shape[1]
dq_max = np.ones(dof) * 4  # joint velocity limits
ddq_max = np.ones(dof) * 20  # joint acceleration limits
dddq_max = np.ones(dof) * 100  # joint jerk limits

# TrajectoryP4 need the timestamps for each waypoint is known
estimated_duration = 10.0
ts = np.r_[0, np.add.accumulate(np.linalg.norm(qs[1:] - qs[:-1], axis=1))]
ts *= (estimated_duration / ts[-1])

# change the default values if needed
options = TrajectoryRnP4.Options()
options.polynomial_order = 6
options.derivative_order = 3
options.continuity_order = 2
options.num_sample_points_each_segment = 10
options.max_iters = 10000
options.time_limits = 10.0
t0 = time.time()
traj3 = TrajectoryRnP4([Rx(list(q)) for q in qs], ts,
                       dq_max, ddq_max, dddq_max, options)
print("cost time: ", time.time() - t0)
PlotTraj(traj3, dq_max, waypoints=qs,
         title="TrajectoryP4 (based on estimated duration)")
