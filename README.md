## Environment:
ros2 humble, YOLO v8, ubuntu 22.04, gazebo11, rviz2


## Dependencies:
- lcm (Needs to be built from source, instructions [here](https://lcm-proj.github.io/lcm/))
- [navigation2](https://github.com/ros-navigation/navigation2)
- [ros2_control](https://github.com/ros-controls/ros2_control)
- [ros2_controllers](https://github.com/ros-controls/ros2_controllers)
- [gazebo_plugins](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_plugins)

## Replacement by the user

1. Replace the paths on lines 6 and 7 in this [file](./go1_sim/go1_navigation/params/nav2_params.yaml) with your workspace paths.
2. Replace the path on line 14 in go1_robot/go1_detection/go1_detection/yolov8_number_detection_node.py with your workspace path.
(ex.         model_path = '/home/yerynkim/swlab_final/go1_robot/go1_detection/models/best.pt')

## Implementation Steps

1. ros2 launch go1_gazebo spawn_go1.launch.py

2. ros2 run unitree_guide2 junior_ctrl **Run this once the last controller plugin (*RL_calf_controller*) has loaded successfully**
   : Once the costmaps are visible in rviz, switch the mode in *window 2* by pressing '2'. The robot will stand up. Switch to nav mode by pressing '5'.

3. ros2 launch go1_navigation navigation.launch.py

4. ros2 run teleop_twist_keyboard teleop_twist_keyboard
   or -> ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -r 10 

5. ros2 run go1_detection yolov8_number_detection_node

## Optional - Training 
If you want to get different detection model, follow these steps:
1.Run generate_images.py to create images and labels. 
2.For training dataset, copy these to /Detection/dataset/train. For validation dataset, generate separately again by running python file, and copy some of images and labels to /Detection/dataset/val.
3.For training phase, upload Detection folder to Google Drive-> open folder, right click on detection.ipynb -> open with Google Colab -> Run all the cells.
4.Copy runs/detect/train/weights/best.pt to the models/.


## Remarks
The yolo model should be trained with a bigger dataset in order to improve accuracy.





