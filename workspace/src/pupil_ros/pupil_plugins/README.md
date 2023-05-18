# Darknet-ROS Visualization in Pupil Capture
A plugin for pupil-capture software to overlay bounding boxes from darknet-ros onto pupil-capture's screen. 

## Installing Pupil-Core from Source
Because we need to use custom ros messages, we must run pupil-capture from source. Follow the instructions [here](https://gitlab.lrz.de/-/ide/project/000000000149604E/predictive-exoskeleton/edit/master/-/workspace/src/pupil_ros/pupil_plugins/) for detailed instructions on the installation of pupil-capture from source. 

It is advised to use a virtual environment for the python packages. To assure compatibility with ROS, the following is recommended:
~~~
cd ~/../pupil
virtualenv --system-site-packages venv
source venv/bin/activate
pip install --ignore-installed -r requirements.txt
~~~

## Preparing ROS Environment
For the plugin to find custom ROS messages, you must source a workspace that has the messages/packages you would like to use. For this plugin, we need the custom ROS messages from the darknet_ros_msgs package. You can of course source an already existing workspace but for an easier setup, it helps to create an additional workspace within the plugin directory of the pupil software. A workspace with the required ROS messages is provided under `/predictive-exoskeleton/workspace/src/pupil_ros/pupil_plugins/ros_msgs`. Copy it to the following location:

    ~/.../pupil/capture_settings/plugins/

And then build:

    cd ros_msgs
    catkin_make

## Linking for Version Control
The plugin is written as a python script and must be placed in the plugin folder of pupil-capture. This makes version control difficult unless you place the source code of pupil-capture within your repository as well. It is preferable to create a symbolic link of the plugin within the repository to the plugin directory of pupil-capture:

    cd ~/../pupil/capture_settings/plugin/
    ln -s ~/../predictive-exoskeleton/workspace/src/pupil_ros/pupil_plugins/darknet_plugin.py

With this you can track changes you make while still being able to use the plugin.

## Starting Pupil-Capture
To properly start the software with all required dependencies:

    cd ~/pupil
    source venv.bin/activate
    source capture_settings/plugins/ros_msgs/devel/setup.bash
    python pupil_src/main.py capture

## Using the Plugin
1. In a terminal run `roscore`:
    
        roscore

2. Start pupil-capture with the above steps. 
3. Follow the steps in the main readme to start pupil_stream and darknet_ros. 
4. Calibrate your gaze.
5. In pupil-capture, go to Plugin Manager and activate Darknet Visualizer.
6. Click on the icon for Darknet Visualizer. 
7. Click on 'Initialize ROS Node'.
8. To see all bounding boxes, select 'All Bounding Boxes'. If you wish to only see the boxes that is being watched by the user, select 'Fixated Bounding Boxes'
9. Click Subscribe.
    * Selecting both or none will return an error when subscribing. please only select one. 

## TODO
To streamline the process, it would be best to turn the primary ROS connector into a plugin as well. There are currently no plans to add this feature.