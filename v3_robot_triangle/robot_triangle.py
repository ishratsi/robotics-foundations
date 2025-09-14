import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
# initial position and orientation of the robot (dot)
x, y, theta = 0, 0, 0

#initialize lists for path followed by the robot
xs, ys = [], []

# linear velocity
v = 1
# angular velocity
w = 0.2
# time step
dt = 0.1  

def update(frame):   # update the robot's position
    global x, y, theta, xs, ys

    # update position
    theta +=  w * dt
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt

    # store the current position
    xs.append(x)
    ys.append(y)

    # Create the triangle shape of the robot in local frame (facing right at initiaition)
    robot_shape = np.array([
        [0.5, 0],     # tip
        [-0.25, 0.35], # back left
        [-0.25, -0.35],# back right
        [0.5, 0]      # close shape
    ])

    # Rotation matrix
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])

    # Rotate and translate the triangle to the robot's current position and orientation
    ### R.T is transpose of R and @ represents matrix multiplication
    rotated = robot_shape @ R.T  
    translated = rotated + np.array([x, y])


    # step 1: Clear the old drawing
    plt.cla()

    # step 2: Set the axes limits
    plt.xlim(-10, 20)
    plt.ylim(-10, 20)

    # step 3: Draw the robot triangle after rotation and translation
    plt.fill(translated[:, 0], translated[:, 1], 'r')

    # step 4: draw the path followed by the robot
    plt.plot(xs, ys, 'b-')

# Animate the robot movement
fig = plt.figure()
ani = animation.FuncAnimation(fig, update, frames=200, interval=100)
plt.show()