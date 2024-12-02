# README: Learning Robotics with iRobot Create Simulator Toolbox

## **Foundation of Robotics**
This folder is a collection of demonstrations focused on the fundamentals of robotics using the **iRobot Create Simulator Toolbox**. It aims to help learners understand essential robotics concepts, including sensor integration, obstacle avoidance, path planning, and multi-sensor navigation. Each demo builds upon the previous one to progressively introduce more advanced topics while ensuring a solid grasp of basic robotics concepts.

### **Contents**

#### **Demo-1: Basic Motion and Sonar Integration**
- **Overview**:
  - Introduces the basic integration of motion commands with simple obstacle detection using the **Sonar** sensor.
- **Files**:
  - **ExampleMap_MG.txt**: Defines the environment for the robot, including the walls, reference lines, beacons, and virtual walls for simulation.
  - **TestProg_1.m**: MATLAB script to control a robot that combines direct movement commands with sonar-based feedback for obstacle detection.
- **Limitations**:
  - **Blind Movements**: Initial robot movements may be blind to the environment, causing crashes in complex layouts such as `ExampleMap_MG.txt`.
  - **Limited Sensor Coverage**: Only front-facing sonar detection is available, leading to blind spots for obstacles to the side or behind.
  - **No Path-Planning**: Lacks the ability to navigate to specific targets or implement mapping logic.

#### **Demo-2: Enhanced Obstacle Avoidance (Modified Demo-1)**
- **Overview**:
  - Demonstrates advanced integration of motion commands with real-time obstacle detection and avoidance using **Sonar**.
- **Files**:
  - **ExampleMap_MG.txt**: The map file defines the environment for robot simulations, including walls, reference lines, and potential trap zones like corner traps.
  - **TestProg_1.m**: A refined version of the initial script, enhancing the robot's ability to navigate safely using sonar feedback and dynamic obstacle avoidance.
- **Improvements**:
  - **Real-Time Sensor Monitoring**: Continuous sonar monitoring allows the robot to halt and avoid obstacles dynamically during forward movement and turns.
  - **Enhanced Obstacle Avoidance**: Obstacle avoidance logic has been added, including backing up, turning, and resuming motion, allowing the robot to navigate around complex obstacles.
  - **Modular Functions**: The addition of `navigateWithSensors`, `turnWithSensors`, and `avoidObstacle` modular functions makes the code more adaptable and reusable for complex tasks.

#### **Demo-3: Experimental Navigation with Parameter Tuning**
- **Overview**:
  - An experimental demo focusing on the effects of different motion parameters and enhancing obstacle avoidance using **Sonar** sensors.
  - Designed to explore the effects of parameter tuning and introduce experimental obstacle courses for deeper learning.
- **Files**:
  - **TestProg_Exp.m**: MATLAB script for experimenting with different speed values, safe distances, and turning angles to improve navigation and obstacle avoidance.
- **Improvements**:
  - **Parameter Tuning for Motion**: Experiments with different speeds and safe distances, allowing learners to see their impact on robot navigation and obstacle avoidance.
  - **Advanced Obstacle Avoidance**: Uses varied turn angles (e.g., 45°, 90°, 135°) to clear obstacles, enhancing the robot's ability to deal with tight spaces and complex layouts.
  - **Obstacle Course Simulation**: Implements a sequence of pre-defined movements to navigate a simulated obstacle course, testing decision-making in real-time scenarios.
  - **Modular Design**: Refined modular functions (`navigateWithSensorsExperiment`, `turnWithSensorsExperiment`, `avoidObstacleExperiment`) increase flexibility, making the code easy to adapt and extend for further experiments.
  - **Informative Logging**: Detailed console logging provides insights into sensor readings and decisions, aiding in debugging and performance analysis.

### **Getting Started**
- **Requirements**:
  - MATLAB and the **iRobot Create Simulator Toolbox**.
  - Basic understanding of MATLAB scripting and robotics concepts such as obstacle avoidance and sensor feedback.
- **Running a Demo**:
  1. **Load the Map File** (`ExampleMap_MG.txt` or `multi_sensor_map.txt`) in the simulator.
  2. **Run the Corresponding Script** (`TestProg_1.m`, `TestProg_Exp.m`) in MATLAB.
  3. **Observe Robot Behavior** and adjust parameters to see how changes affect navigation.

### **Learning Objectives**
- **Demo-1**: Understand basic motion commands and sensor integration.
- **Demo-2**: Learn real-time sensor monitoring, dynamic obstacle avoidance, and modular code structure.
- **Demo-3**: Explore the impact of different parameters, experiment with obstacle courses, and understand the importance of adaptive logic.

### **Further Improvements and Challenges**
- **Add Multi-Sensor Integration**: Introduce other sensors like bump and cliff sensors to enhance environment awareness.
- **Implement Path-Planning Algorithms**: Introduce algorithms like A* for more sophisticated navigation towards target points.
- **Expand Map Complexity**: Add more dynamic elements or moving obstacles to simulate more realistic challenges.

### **Contact and Contributions**
For contributions, suggestions, or questions, feel free to create an issue or submit a pull request. This project is a learning tool, and contributions to improve its educational value are always welcome.

Happy learning, and enjoy exploring the world of robotics!

