## Jin-Ho Lee Master Thesis: Predictive Exoskeleton

*Package sources: [darknetros](https://github.com/leggedrobotics/darknet_ros) and [pupil-ros](https://gitlab.lrz.de/000000000149604E/pupilros).

### Quick Demo Eye Tracking + Object Detection

1. Connect pupil core to PC via USB
2. Open pupil capture
3. Make sure Network API is on under PLugin Manager
4. Make sure Port is set to 50020 under Network API
5. Make sure format is set to BGR under Network API
6. Perform gaze calibration

- Terminal 1: roscore
- Terminal 2:
    - source python_packages (if virtual environment)
    - source devel/setup.bash
    - roslaunch pupil_connect pupil_stream.launch
- Terminal 3:
    - source devel/setup.bash
    - roslaunch darknet_ros darknet_ros.launch


### Starting exoskeleton + experiment nodes

1. roscore
2. roslaunch skin_control skin_driver.launch
3. roslaunch skin_control skin_control.launch
4. rosrun rosserial_python serial_node.py /dev/ttyACM0
5. roslaunch exo_control exo_control.launch
6. roslaunch experiment experiment.launch
7. rosrun plotjuggler plotjuggler


### Object Holding

1. Terminal 1: roscore
2. Terminal 2:
    - cd ~/pupil
    - source pupil_v/bin/activate
    - source capture_settings/plugins/ros_msgs/devel/setup.bash
    - python pupil_src/main.py capture
3. Adjust eye and world cameras.
4. Move eyes around until 3D model fits eyeballs. Reset if need be. Freeze model once satisfied
5. Setup Network API like above
6. Terminal 3:
    - source .../venv/bin/activate
    - source .../devel/setup.bash
    - roslaunch pupil_connect object_detector.launch
7. Terminal 4:
    - source .../venv/bin/activate
    - source .../devel/setup.bash
    - rosrun pupil_connect hand_detector.py
8. Terminal 5:
    - source .../venv/bin/activate
    - source .../devel/setup.bash
    - roslaunch pupil_connect object_watching.launch
9. In pupil capture, activate Darknet Visualzer from Plugin Manager
10. Click 'Initialize ROS node'
11. Click 'Subscribe'
12. To choose whether all detected bounding boxes are shown, select 'All Bounding Boxes' or 'Fixated Bounding Boxes' and click 'Change'


### Unloading Experiment

1. Terminal 1: roscore
2. Terminal 2: roslaunch skin_control skin_driver.launch
3. Terminal 3: roslaunch skin_control skin_control.launch
4. Terminal 4: rosrun rosserial_python serial_node.py /dev/ttyACM0
5. Terminal 5: roslaunch exo_control exo_control.launch
    - Type q to quit node
    - Type c to activate/deactivate upper sensors
    - Type p to activate/deactivate predictive load compensation
6. Terminal 6: roslaunch experiment unloading.launch
    - Press 'Enter' to start experiment
    - Press 'Enter to iterate trial