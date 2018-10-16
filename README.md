## Robot Localization
This is the base repo for the Olin Computational Robotics warmup project

Gretchen Rice // Isaac Vandor // Comprobo 18

For code, please go to: [robot_localizer](https://github.com/isaacvandor/robot_localization/tree/master/robot_localizer/scripts)

For media, please go to: [Media folder](https://github.com/isaacvandor/robot_localization/tree/master/media)

For rosbags, please go to: [ROS bags](https://github.com/isaacvandor/robot_localization/tree/master/robot_localizer/bags)

For project writeup, please go to: [Project Writeup](https://github.com/isaacvandor/robot_localization/blob/master/writeup.md)

To run code with bag file, start roscore then run command roslaunch robot_localizer test_bagfile.launch map_name:=*ac109_1* where *ac109_1* can be replaced with a different map

To run with live robot movement, start roscore then run command roslaunch robot_localizer test_live.launch map_file:=*mymap.yaml* where again *mymap.yaml* should be replaced with the name of the map you are driving the robot around (Note: you may have to create a new map for this. See https://sites.google.com/site/comprobo18/projects/robot-localization?authuser=0 for more information about creating a map).
