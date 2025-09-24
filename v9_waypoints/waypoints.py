import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

# ----------------------------
# Robot Class
# ----------------------------
class Robot:
    def __init__(self, x=0, y=0, theta=0, v=0, w=0, dt=0.1):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.w = w
        self.dt = dt

    def update(self):
        # Update position using unicycle model
        self.theta += self.w * self.dt
        self.x += self.v * np.cos(self.theta) * self.dt
        self.y += self.v * np.sin(self.theta) * self.dt

# ----------------------------
# Initialize robot and state
# ----------------------------
robot = Robot()
xs, ys = [], []   # path history

# ----------------------------
# Waypoints
# ----------------------------
waypoints = [(2, 1), (6, 4), (3, 7), (-2, 5), (-4, 0), (0, -3)] # list of (x, y) tuples
current_goal_index = 0

# ----------------------------
# Control logic
# ----------------------------
def control(frame):
    global current_goal_index
    # If all waypoints are reached, stop the robot
    if current_goal_index >= len(waypoints):
        robot.v = 0
        robot.w = 0
        return
    
    goal = waypoints[current_goal_index]

    dx = goal[0] - robot.x
    dy = goal[1] - robot.y
    distance = np.hypot(dx, dy)

    desired_theta = np.arctan2(dy, dx)
    theta_error = desired_theta - robot.theta
    
    # Normalize angle to [-pi, pi]
    theta_error = (theta_error + np.pi) % (2 * np.pi) - np.pi


    k_w = 1.0

    robot.w = k_w * theta_error
    if abs(theta_error) < np.pi/12:
        robot.v = 1
    else:
        robot.v = 0.1

    if distance < 0.1:
        current_goal_index += 1

    pass

# ----------------------------
# Update per frame
# ----------------------------
def update(frame):
    global current_goal_index

    control(frame)      # set robot.v, robot.w
    robot.update()      # update robot state

    xs.append(robot.x)
    ys.append(robot.y)

    plt.cla()
    plt.xlim(-5, 15)
    plt.ylim(-5, 15)

    # Draw robot (triangle)
    robot_shape = np.array([[0.5, 0], [-0.25, 0.35], [-0.25, -0.35], [0.5, 0]])
    R = np.array([[np.cos(robot.theta), -np.sin(robot.theta)],
                  [np.sin(robot.theta),  np.cos(robot.theta)]])
    rotated = robot_shape @ R.T
    translated = rotated + np.array([robot.x, robot.y])
    plt.fill(translated[:, 0], translated[:, 1], 'r')

    # Draw path
    plt.plot(xs, ys, 'b-')

    # Draw waypoints
    for wp in waypoints:
        plt.plot(wp[0], wp[1], 'go')

# ----------------------------
# Animate
# ----------------------------
fig = plt.figure()
ani = animation.FuncAnimation(fig, update, frames=200, interval=100)
plt.show()