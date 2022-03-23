import numpy as np
from sys import stdin
import matplotlib.pyplot as plt
import math

header = stdin.readline()
hdrs = [s.strip() for s in header.strip().split(",")]
assert hdrs == ["t", "x", "y", "theta", "xdot", "ydot", "thetadot"]

t = []
x = []
y = []
theta = []
xdot = []
ydot = []
thetadot = []
for line in stdin:
    cells = [float(s.strip()) for s in line.strip().split(",")]
    t.append(cells[0])
    x.append(cells[1])
    y.append(cells[2])
    theta.append(cells[3])
    xdot.append(cells[4])
    ydot.append(cells[5])
    thetadot.append(cells[6])


# x vs y up dot, with theta arrows

# x vs y vs theta vs time

npxdot = np.array(xdot)
npydot = np.array(ydot)
nptdot = np.array(thetadot)

ROBOT_R_METER = 0.520855

maxvel = np.sqrt(npxdot**2 + npydot**2) + np.abs(nptdot) * ROBOT_R_METER


plt.subplot(2, 1, 1)
plt.plot(x, y, linewidth=3)
for xx,yy,tt in zip(x, y, theta):
    dx = 0.05 * math.cos(tt)
    dy = 0.05 * math.sin(tt)
    plt.arrow(xx, yy, dx, dy, width=0.01, color="red", head_width=0.02, head_length=0.02)
plt.xlabel('x')
plt.ylabel('y')



plt.subplot(2, 4, 5)
plt.plot(t, x)
plt.xlabel('t')
plt.ylabel('x')


plt.subplot(2, 4, 6)
plt.plot(t, y)
plt.xlabel('t')
plt.ylabel('y')


plt.subplot(2, 4, 7)
plt.plot(t, theta)
plt.xlabel('t')
plt.ylabel('theta')



plt.subplot(2, 4, 8)
plt.plot(t, maxvel)
plt.xlabel('t')
plt.ylabel('max wheel vel')

# plt.tight_layout()

plt.show()
