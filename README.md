# Jin-Ho Lee Master Thesis: Predictive Exoskeleton

## Project Description
**Thesis Title:** Vision-based predictive motion control of an assistive elbow exoskeleton

The goal of this project is the integration of the pupil core eye tracker from Pupil Labs with an assistive elbow exoskeleton developed at the Institute of Cognitive Systems of the Technical University of Munich for the predictive identification of human intention to pick up objects to thus assist more efficiently with the exoskeleton.

### The Exoskeleton
The exoskeleton consists of 3D printed components and a servomotor for motor control and assistance. The servomotor is connected to an Arduino Uno, which in turn is connected to a PC and communicates with other software components (primarily the eye tracker) through ROS. 

 ![experimenta_setup](https://github.com/Cyberjin7/master_thesis/assets/40031638/4cc2867e-856d-40fc-ae04-6c02abbbce81)

### The Eye Tracker

![pupilcore](https://github.com/Cyberjin7/master_thesis/assets/40031638/19e6381d-cbbd-4c52-b550-c99700c81f88)

Pupil Core is an eye tracker with 3 cameras: 1 camera for each eye and a front facing world camera. Through software provided by Pupil Labs called Pupil Capture, it is possible to receive the camera streams in real time with a connected PC. Upon calibration through Pupil Capture, the gaze of the user can also be tracked on the monitor of the connected PC. Pupil Capture provides a networking plugin via ZeroMQ through which communication with other software is possible. Because the network plugin uses a Pub-Sub Pattern, it synergizes well with ROS. 

With access to the camera streams, YOLO is used to identify objects seen by the world camera and MediaPipe is used to track the hand of the exoskeleton user. Using this information, coupled with common human behavior to look at potential objects of interaction, it is possible to identify intent to pick up a given object. 

![prediction_successful](https://github.com/Cyberjin7/master_thesis/assets/40031638/b03a344c-7a7c-4729-8379-573bfb02b0ea)


### Why is this useful?
From advances in neuroscience, it is now known that most human behavior is either automated or predictive in nature, particularly motor control of the human body happens predictively. Studies suggest that during object interactions, prior to picking up an object, humans estimate the mass of the target object and predictively adjust the amount of force they will use to grab and pick up said object. ([Source](https://doi.org/10.1016/j.conb.2006.10.005))

Using an eye tracker, it is now possible to imitate this behaviour. For the scope of this project, a table of common masses for daily objects are provided to the exoskeleton. Using the eye tracker to predict the object a human intends to pick up, the assistive torque provided by the exoskeleton is adjusted to take into account the mass that will be found in the user's hand soon. An overview is visualized below:

![overview_clean](https://github.com/Cyberjin7/master_thesis/assets/40031638/d32a3650-1b4f-453f-946d-24f9ad5d274f)


## Project Structure

- **analysis**: Contains data collected from experiments. Has jupyter notebooks for calculating dynamic equations via Euler-Lagrange Iteration and data analysis noteboboks.
- **arduino**: Contains code for Arduino Uno to move servomotor position from ROS messages from PC. 
- **workspace**: Contains ROS packages
    - **darknet_ros**: YOLO for ROS.
    - **exo**: package for controlling motor postion. Dynamic equation calculates done here.
    - **plotter**: package for nodes for running experiments.
    - **pupil_ros**: package for connecting Pupil Capture with ROS. 

## Installation

The project was done in **Ubuntu 20.04** with **ROS Noetic**. Compatibility with other distributions is not guaranteed. 

The darknet ROS package is used for YOLO integration into ROS. This requires additional setup. Instructions can be found [here](https://github.com/leggedrobotics/darknet_ros).  
*The use of a GPU is recommended.

Specifics on using Pupil Capture with ROS is provided [here](https://github.com/Cyberjin7/pupil_ros).

## Demos

### Quick Demo Eye Tracking + Object Detection
Quick steps to get pupil capture running with YOLO.
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


### Starting Exoskeleton + Experiment Nodes
Preliminary Experiment whether user feels a difference in assistance from the servomotor.
1. roscore
2. roslaunch skin_control skin_driver.launch
3. roslaunch skin_control skin_control.launch
4. rosrun rosserial_python serial_node.py /dev/ttyACM0
5. roslaunch exo_control exo_control.launch
6. roslaunch experiment experiment.launch
7. rosrun plotjuggler plotjuggler


### Visualize Object Holding Instructions
Instructions to start object detection and hand detection nodes and visualize these directly in Pupil Capture's UI. 
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
9. In pupil capture, activate Darknet Visualizer from Plugin Manager
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
