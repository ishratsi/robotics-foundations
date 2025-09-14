# Robotics Foundations 🚀

This repository documents my learning journey in robotics software through small, incremental simulations.  
Each version adds one new feature, starting from a simple moving dot and progressing toward autonomous robot behaviors.

## Versions

- **v1: Moving Dot**  
  Basic motion model using linear and angular velocity.

- **v2: Trail**  
  Added path visualization.

- **v3: Triangle Robot**  
  Robot represented as a triangle showing orientation.

- **v4: Keyboard Control**  
  Arrow keys to move and turn, spacebar to stop.

- **v5: Robot Class**  
  Refactored into a `Robot` class for modularity.

- **v6: Dynamic Keyboard Control**  
  Velocity and turn rate now change incrementally (constant acceleration).

## Next Steps
- v7: Obstacles and collision detection  
- v8: Autopilot paths (square, circle, figure-8)  
- v9: Simulated sensors (rangefinder, lidar)  
- v10: Path planning (A*, obstacle avoidance)

---

### How to Run
1. Clone this repo:
   ```bash
   git clone https://github.com/ishratsi/robotics-foundations.git
   cd robotics-foundations
2. Enter any version folder and run the Python script:
        cd v1_moving_dot
        python3 moving_dot.py
    Replace v1_moving_dot with any version folder you want to try (e.g. v6_dynamic_keyboard_control).

---

## Preview

### v1: Moving Dot
![v1 demo](media/v1_sim.gif)

### v2: Moving Trail
![v2 demo](media/v2_sim.gif)

### v3: Triangle Robot
![v1 demo](media/v3_sim.gif)
