<h2>Setup</h2>
Clone this package in your ROS workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Also install catkin build (use catkin build instead of catkin_make): https://catkin-tools.readthedocs.io/en/latest/installing.html



<h2>Basic ROS Usage</h2>

### NEED TO START LIKE THIS NOW ON MAIN BRANCH
1. Open 2 terminals and plug in webcam.
2. Enter ```roslaunch calibration testbed_calibration.launch```
3. In second terminal either use ```rqt --standalone calibration``` or ```rqt``` if you want to see the camera feed. 


If anything breaks, run:<br>
<code>catkin clear</code><br>
and rebuild the workspace again<br>
<code>catkin build</code>

