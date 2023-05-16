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