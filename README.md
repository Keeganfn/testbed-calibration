Clone this package in your ROS workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Also install catkin build (use catkin build instead of catkin_make): https://catkin-tools.readthedocs.io/en/latest/installing.html



Using ROS:

1. Open 2 terminals
2. Type 'roscore' in one terminal to start the main ROS node
3. To run a node, type rosrun calibration <filename>, 
    where <filename> is your Python script under /calibration/src/

If anything breaks, run:
catkin clear
catkin build

