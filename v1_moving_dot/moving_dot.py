import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

# initial position and orientation of the robot (dot)
x, y, theta = 0, 0, 0

# linear velocity
v = 1
# angular velocity
w = 0.2
# time step
dt = 0.1  

# This function updates the robot's position
def update(frame):   
    global x, y, theta

    # update position
    theta +=  w * dt
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt

    # step 1: clear the old drawing
    plt.cla()

    # step 2: set the axes limits
    plt.xlim(-10, 20)
    plt.ylim(-10, 20)

    # step 3: draw the robot as a dot
    plt.plot(x,y,'ro')  

# Animate the robot movement
fig = plt.figure()
ani = animation.FuncAnimation(fig, update, frames=200, interval=100)
plt.show()