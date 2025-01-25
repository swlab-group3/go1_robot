# **Go1 Robot Navigation and Detection Setup**

## **Environment**
- **OS:** Ubuntu 22.04  
- **Frameworks:** ROS 2 Humble, YOLO v8  
- **Simulators:** Gazebo 11, Rviz 2  

---

## **Dependencies**
Ensure the following dependencies are installed:
1. [LCM](https://lcm-proj.github.io/lcm/) (build from source)  
2. [Navigation2](https://github.com/ros-navigation/navigation2)  
3. [ros2_control](https://github.com/ros-controls/ros2_control)  
4. [ros2_controllers](https://github.com/ros-controls/ros2_controllers)  
5. [gazebo_plugins](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_plugins)  

---

## **Pre-Setup Modifications**
1. **Update Navigation Parameters:**
   - Replace lines 6 and 7 in the file:  
     `./go1_sim/go1_navigation/params/nav2_params.yaml`  
     with your workspace paths.

2. **Update YOLOv8 Model Path:**  
   - Modify line 14 in:  
     `go1_robot/go1_detection/go1_detection/yolov8_number_detection_node.py`  
     Example:  
     ```python
     model_path = '/home/your_username/workspace/go1_robot/go1_detection/models/best.pt'
     ```

---

## **Implementation Steps**

### 1. **Spawn the Robot**
Launch the robot in Gazebo and Rviz:  
```bash
ros2 launch go1_gazebo spawn_go1.launch.py

### 2. **Start the Controller**
Run this command after the **RL_calf_controller** has successfully loaded:  
```bash
ros2 run unitree_guide2 junior_ctrl
