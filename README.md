
STEP 1: Install ROS2 + Dependencies

STEP 2: Create ROS2 Workspace

STEP 3: Create Package

STEP 4: Create Python Node File


Paste this FULL CODE:

Running commands 

cd ~/robot_ws
colcon build --packages-select line_detector
source install/setup.bash
ros2 launch line_detector line_detect.launch.py

If any changes need to be done in the code then run this code :
nano ~/robot_ws/src/line_detector/line_detector/line_detector_node.py

