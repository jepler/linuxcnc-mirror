#!/usr/bin/env python3
import pyjson5

import sys
import re
from matplotlib import pyplot as plt
import numpy as np

axes={
    'X': 0,
    'Y': 1,
    'Z': 2,
    'A': 3,
    'B': 4,
    'C': 5,
    'U': 6,
    'V': 7,
    'W': 8,
}

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(sys.stderr, 'No logfile specified')
        exit(1)

    logs = []
    with open(sys.argv[1], 'r') as log_file:
        for l in log_file.readlines():
            # Try to filter out obviously non-JSON lines
            if re.search('{', l) is None or re.search('}', l) is None:
                continue
            try:
                logs.append(pyjson5.decode(l))
            except Exception as e:
                print('Ignoring exception ', e)

    axis_pos = np.array([e.get('axis_pos', None) for e in logs if e.get('log_entry', None) == 'tpRunCycle'])
    axis_vel = np.array([e.get('axis_vel', None) for e in logs if e.get('log_entry', None) == 'tpRunCycle'])
    axis_accel = np.array([e.get('axis_accel', None) for e in logs if e.get('log_entry', None) == 'tpRunCycle'])
    times = np.array([e.get('time', None) for e in logs if e.get('log_entry', None) == 'tpRunCycle'])

    np.savetxt('axis_pos.csv', axis_pos, delimiter=',')
    np.savetxt('axis_vel.csv', axis_vel, delimiter=',')
    np.savetxt('axis_accel.csv', axis_accel, delimiter=',')

    plt_axis = 'X'
    idx = axes[plt_axis]

    plt.figure(1)
    plt.plot(times * 1000.0, axis_pos[:, idx])
    plt.grid(True)
    plt.title('{} Axis position vs time, ms'.format(plt_axis))

    plt.figure(2)
    plt.plot(times * 1000.0, axis_vel[:, idx])
    plt.grid(True)
    plt.title('{} velocity vs time, ms'.format(plt_axis))

    plt.figure(3)
    plt.plot(times * 1000.0, axis_accel[:, idx])
    plt.grid(True)
    plt.title('{} Axis acceleration vs time, ms'.format(plt_axis))

    plt.figure(4)
    plt.plot(times * 1000.0, np.sqrt(np.sum(np.square(axis_vel[:, 0:2]), 1)))
    plt.grid(True)
    plt.title('XYZ velocity vs time, ms')

    plt.show()



