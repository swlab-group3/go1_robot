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
ros2 launch go1_gazebo spawn_go1.launch.py


### 2. Start the Controller

Run this command after the RL_calf_controller has successfully loaded:

ros2 run unitree_guide2 junior_ctrl

### 3. Switch Robot Modes

    Wait for the costmaps to appear in Rviz.
    In Window 2, switch the robot modes:
        Press 2: The robot will stand up.
        Press 5: Switch to navigation mode.

### 4. Run Navigation

Launch the navigation file:

ros2 launch go1_navigation navigation.launch.py

### 5. Manual Control

Choose one of the following control methods:

    Keyboard Control:
    Use the keyboard to control the robot:

ros2 run teleop_twist_keyboard teleop_twist_keyboard

Joystick Control:
Use a joystick to control the robot:

ros2 run joy joy_node

Velocity Commands:
Publish velocity commands directly:

    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -r 10

### 6. Run YOLOv8 Object Detection

Start the YOLOv8 detection node:

ros2 run go1_detection yolov8_number_detection_node

### 7. Test Object Detection in Simulation

    Place an object (e.g., number box, person, construction cones) in front of the robot in Gazebo.
    Manually control the robot to check if it recognizes the object as it moves.

## Optional: Training a Custom YOLOv8 Model

Follow these steps to train a new YOLOv8 model:

    Run the generate_images.py script to create images and labels:

    python generate_images.py

    Copy the generated data to the appropriate dataset folders:
        Training dataset:
        Copy to Detection/dataset/train.
        Validation dataset:
        Run generate_images.py again and copy a portion of the data to Detection/dataset/val.

    For training:
        Upload the Detection folder to Google Drive.
        Open the folder, right-click on detection.ipynb, and select Open with Google Colab.
        Run all the cells in the notebook.

    Once training is complete:
        Copy the trained model file (best.pt) from runs/detect/train/weights/ to the models/ folder.

## Remarks

    To improve accuracy, train the YOLO model with a larger dataset.
