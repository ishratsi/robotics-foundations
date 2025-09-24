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
waypoints = [(2, 1), (6, 4), (3, 7), (-2.5, 4), (-4, 0), (0, -3)] # list of (x, y) tuples
current_goal_index = 0

# ----------------------------
# Control logic
# ----------------------------
def control(frame, state='drive', avoid_theta=0):
    global current_goal_index
    
    # If all waypoints are reached, stop the robot
    if current_goal_index >= len(waypoints):
        robot.v = 0
        robot.w = 0
        return

    if state == 'turn':
        # Emergency: turn away from obstacle
        robot.w = 2.0 * avoid_theta
        robot.v = 0  # stop moving forward
    
    elif state == 'avoid':
        # Avoidance: turn towards escape direction
        theta_error = avoid_theta - robot.theta
        theta_error = (theta_error + np.pi) % (2 * np.pi) - np.pi
        robot.w = 2.0 * theta_error
        robot.v = 0.5  # slow forward movement

    else:
        state == 'drive'
        # Normal waypoint following
        goal = waypoints[current_goal_index]

        dx = goal[0] - robot.x
        dy = goal[1] - robot.y
        distance = np.hypot(dx, dy)

        desired_theta = np.arctan2(dy, dx)
        theta_error = desired_theta - robot.theta

        # Normalize angle to [-pi, pi]
        theta_error = (theta_error + np.pi) % (2 * np.pi) - np.pi

        k_w = 1.5

        robot.w = k_w * theta_error
        if abs(theta_error) < np.pi/6:
            robot.v = 1
        else:
            robot.v = 0.2

        if distance < 0.2:
            current_goal_index += 1

    pass

# ----------------------------
# Update per frame
# ----------------------------
def update(frame):
    global current_goal_index

    # Walls
    walls = [(-2,2,3,5), (4,6,1,3), (8,10,5,7)] # (x_min, x_max, y_min, y_max)

    # Computing next position to check for collision
    next_x = robot.x + robot.v * np.cos(robot.theta) * robot.dt
    next_y = robot.y + robot.v * np.sin(robot.theta) * robot.dt
    
    state = 'drive'
    avoid_theta = 0

    # Approximate radius of the robot for collision detection and buffer for safety
    robot_radius = 0.3
    avoid = 0.3
    buffer = 0.1

    # Simple collision detection with walls
    for wall in walls:
        x_min, x_max, y_min, y_max = wall
        if next_x + robot_radius + avoid >= x_min and next_x - robot_radius - avoid <= x_max and next_y + robot_radius + avoid >= y_min and next_y - robot_radius - avoid <= y_max:
            # Collision detected, stop the robot
            state = 'avoid'
            # Determine which way to turn based on shortest escape route
            dist_to_top = abs(robot.y - y_max)
            dist_to_bottom = abs(robot.y - y_min)
            dist_to_left = abs(robot.x - x_min)
            dist_to_right = abs(robot.x - x_max)

            min_dist = min(dist_to_top, dist_to_bottom, dist_to_left, dist_to_right)
            
            if min_dist == dist_to_top:
                avoid_theta = np.pi/2  # turn up
            elif min_dist == dist_to_bottom:
                avoid_theta = -np.pi/2  # turn down
            elif min_dist == dist_to_left:
                avoid_theta = np.pi  # turn left
            else:
                avoid_theta = 0  # turn right
            
            break

        if next_x + robot_radius + buffer >= x_min and next_x - robot_radius - buffer <= x_max and next_y + robot_radius + buffer >= y_min and next_y - robot_radius - buffer <= y_max:
            # Collision detected, stop the robot
            state = 'turn'

            # Emergency: turn away from wall center
            wall_center_x = (x_min + x_max) / 2
            wall_center_y = (y_min + y_max) / 2
            avoid_theta = np.arctan2(robot.y - wall_center_y, robot.x - wall_center_x)
            break

    # set robot.v, robot.w based on collision
    control(frame, state, avoid_theta)

    # update robot state
    robot.update()

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

    ## Adding a rectangle to represent the wall at at walls defined above
    for wall in walls:
        x_min, x_max, y_min, y_max = wall
        plt.fill([x_min, x_max, x_max, x_min], [y_min, y_min, y_max, y_max], 'g', alpha=0.5)

# ----------------------------
# Animate
# ----------------------------
fig = plt.figure()
ani = animation.FuncAnimation(fig, update, frames=200, interval=100)
plt.show()