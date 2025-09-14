import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

# This code simulates a simple robot moving in a 2D plane.

# Robot Class defines the robot in the 2D space
class Robot:
    def __init__(self, x=0, y=0, theta=0, v=0, w=0, a_v=0, a_w=0, dt=0.1):
        # Initialize the robot's position and orientation,x and y are coordinates and theta is the angle in radians; 
        # v is linear velocity and w is angular velocity; a_v is linear acceleration and a_w is angular acceleration;
        # dt is time step
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.w = w
        self.a_v = a_v
        self.a_w = a_w
        self.dt = dt

    def update(self):
        # Update the robot's position based on its velocities
        self.theta += self.w * self.dt
        self.x += self.v * np.cos(self.theta) * self.dt
        self.y += self.v * np.sin(self.theta) * self.dt

# Initialize a Robot named robot with default parameters
robot = Robot(a_v = 0.1, a_w = 0.1)

# Lists to store the robot's path
xs, ys = [], [] 


def update(frame):   # Update the robot's position
    robot.update()

    # store the current position
    xs.append(robot.x)
    ys.append(robot.y)

    # Create the triangle shape of the robot in local frame (facing right at initiaition)
    robot_shape = np.array([
        [0.5, 0],     # tip
        [-0.25, 0.35], # back left
        [-0.25, -0.35],# back right
        [0.5, 0]      # close shape
    ])

    # Rotation matrix
    R = np.array([
        [np.cos(robot.theta), -np.sin(robot.theta)],
        [np.sin(robot.theta),  np.cos(robot.theta)]
    ])

    # Rotate and translate the triangle to the robot's current position and orientation
    ### R.T is transpose of R and @ represents matrix multiplication
    rotated = robot_shape @ R.T  
    translated = rotated + np.array([robot.x, robot.y])

    # Now we will draw the robot and its path
    ## step 1: Clear the old drawing
    plt.cla()

    ## step 2: Set the axes limits
    plt.xlim(-10, 20)
    plt.ylim(-10, 20)

    ## step 3: Draw the robot triangle after rotation and translation
    plt.fill(translated[:, 0], translated[:, 1], 'r')

    ## step 4: draw the path followed by the robot
    plt.plot(xs, ys, 'b-')

# This function takes keyboard inputs and sets the linear and angular velocities to some constant value
def on_key(event):
    global v, w
    # Move forward
    if event.key == 'up':
        robot.v += robot.a_v  # Increase linear velocity
    # Move backward
    elif event.key == 'down':
        robot.v += -robot.a_v  # Decrease linear velocity  
    # Turn left
    elif event.key == 'left':
        robot.w += robot.a_w  # Increase angular velocity
    # Turn right
    elif event.key == 'right':
        robot.w += -robot.a_w  # Decrease angular velocity
    # Stop
    elif event.key == ' ':
        robot.v, robot.w = 0.0, 0.0

# Animate the robot movement
fig = plt.figure()
fig.canvas.mpl_connect('key_press_event', on_key)
ani = animation.FuncAnimation(fig, update, frames=200, interval=100)
plt.show()