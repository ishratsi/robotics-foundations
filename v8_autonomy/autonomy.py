import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

# This code simulates a simple robot moving in a 2D plane.

#Ask user if they want manual control or autopilot and store in variable
mode = input("Select mode: 1. Manual Control or 2. Autopilot: ")

# Store the mode in a variable control_mode and default to autopilot if invalid input
try:
    if mode == "1":
        control_mode = "manual"
    elif mode == "2":
        control_mode = "auto"
        try:
            path_choice = input("Choose autopilot path: (1) Circle / (2) Square / (3) Figure-8: ")
            if path_choice == "1":
                autopilot_mode = "circle"
            elif path_choice == "2":
                autopilot_mode = "square"
            elif path_choice == "3":
                autopilot_mode = "figure8"
            else:
                print("Invalid choice, defaulting to circle.")
                autopilot_mode = "circle"
        except Exception as e:
            print(f"Error in autopilot path selection: {e}. Defaulting to circle.")
            autopilot_mode = "circle"
    else:
        raise ValueError("Invalid selection. Input should be 1 for manual or 2 for autopilot.")
except Exception as e:
    print(f"{e} Defaulting to auto control and circle path.")
    control_mode = "auto"
    autopilot_mode = "circle"

# This function takes control_mode and frame number as input and sets the robot's velocities accordingly
def control(control_mode,frame):
    if control_mode == "manual":
        # Manual control logic (e.g., keyboard input)
        pass
    elif control_mode == "auto":
        # Autopilot logic (e.g., predefined path)
        autopilot_control(robot, frame)

# Autopilot logic: circular path, square path, figure-8 path
def autopilot_control(robot, frame):
    if autopilot_mode == "circle":
        robot.v = 1.0  # linear velocity
        robot.w = 0.2  # angular velocity
    elif autopilot_mode == "square":
        # Move forward for N frames, then turn 90°
        cycle = frame % 160  # adjust depending on how long you want each side
        if cycle < 150:      # go straight
            robot.v = 1.0
            robot.w = 0.0
        else:                # turn in place
            robot.v = 0.0
            robot.w = np.pi/2   # ~90° turn

    elif autopilot_mode == "figure8":
        # params you can tweak
        speed = 1.0           # linear speed (m/s)
        omega = 0.3           # turn rate during arcs (rad/s)
        robot.v = speed       # default each frame; we’ll override w per phase

        # derived geometry
        # radius R = v / |omega|
        # straight distance between arc centers ≈ 2R → duration = D / v = (2R)/v = 2/(|omega|)
        turn_frames    = int(np.pi / (abs(omega) * robot.dt))   # half circle (π radians)
        straight_frames = int(2.0 / (abs(omega) * robot.dt))    # cross the center (distance ~ 2R)

        # one full "8" = straight → left half-circle → straight → right half-circle
        period = 2 * straight_frames + 2 * turn_frames
        phase  = frame % period

        if phase < straight_frames:
            # go straight through the center to shift to the other lobe
            robot.w = 0.0
        elif phase < straight_frames + turn_frames:
            # left half-circle
            robot.w = +omega
        elif phase < straight_frames + turn_frames + straight_frames:
            # straight back through the center
            robot.w = 0.0
        else:
            # right half-circle
            robot.w = -omega

    
    # elif autopilot_mode == "figure8":
    #     robot.v = 1.0  # linear velocity
    #     robot.w = 0.3 * np.sin(frame * 0.01)  # oscillating angular velocity for figure-8
    else:
        print("Invalid autopilot mode, defaulting to circle.")

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

# Update the robot's position
def update(frame):
    # Set velocities based on control mode
    control(control_mode, frame)  

    # Adding a wall ((x_min, x_max, y_min, y_max) format)
    walls = []  # no wall for now

    # Computing next position to check for collision
    next_x = robot.x + robot.v * np.cos(robot.theta) * robot.dt
    next_y = robot.y + robot.v * np.sin(robot.theta) * robot.dt
    
    collision = False

    # Approximate radius of the robot for collision detection
    robot_radius = 0.4

    # Simple collision detection with walls
    for wall in walls:
        x_min, x_max, y_min, y_max = wall
        if next_x + robot_radius >= x_min and next_x - robot_radius <= x_max and next_y + robot_radius >= y_min and next_y - robot_radius <= y_max:
            # Collision detected, stop the robot
            robot.v = 0.0
            robot.w = 0.0
            collision = True
            break
    
    # Update the robot's position if no collision
    if not collision:
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

    ## Adding a rectangle to represent the wall at at walls defined above
    for wall in walls:
        x_min, x_max, y_min, y_max = wall
        plt.fill([x_min, x_max, x_max, x_min], [y_min, y_min, y_max, y_max], 'g', alpha=0.5)

# This function takes keyboard inputs and sets the linear and angular velocities to some constant value
def on_key(event):
    if control_mode != "manual":
        return  # Ignore key presses if not in manual mode
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