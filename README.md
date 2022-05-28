[GitHub Repo Link](https://github.com/Keeganfn/testbed-calibration)

<h2>Software Required</h2>
 - A full install of ROS Melodic (Important note, this is only supported on Ubuntu 18.04) - http://wiki.ros.org/melodic/Installation/Ubuntu

### Setup
1. Clone this package in your ROS workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
2. Install catkin build if you do not have it already (use catkin build instead of catkin_make): https://catkin-tools.readthedocs.io/en/latest/installing.html
3. Install the video_stream_opencv package - ```sudo apt-get install ros-melodic-video-stream-opencv```

<h2>USAGE<h2>

## Generic Arm
1. Plug in the Robot Arm and Camera to your computer.
2. Start Arm communication.
3. Start an Rviz environment with the robot arm. 
4. Edit the config.json file in the src/ directory to include the arm base link. (If this step is not completed the calibration will default to the world frame.) 

## Kinova Arm
1. Follow the Kinova setup instructions: https://drive.google.com/file/d/12eizcm0FJI3dnjX7HDMKFX5gtys5OQAb/view?usp=sharing (Written by Nuha Nishat)
2. Make sure you have an RVIZ window with the Kinova arm that mirrors the current Kinova position. 

## Calibration
1. Open 2 terminals and plug in webcam.
2. Enter ```roslaunch calibration testbed_calibration.launch``` 
3. Depending on the workstation and camera you are using you may get an error and need to change the video_device integer parameter in order to correctly recieve the camera feed ```roslaunch calibration testbed_calibration.launch video_device:=<NUMBER>```. Current default is 3 for the reset mechanism testbed. 
4. In second terminal enter ```rqt --standalone calibration```. If you would like to see the camera feed instead run ```rqt``` and select calibration plugin and the image viewer plugin. Set the image viewer plugin to topic ```/camera_1/image_raw```.
5. Follow GUI instructions for Calibration.
6. After you have clicked the button "Display Results" you will need to add the testbed_marker and camera_marker in rviz to see the finished result. You can do this by clicking "add" then "by topic" and selecting the /testbed_marker and /camera_marker topics.


<h2>GUI</h2>

### Important Files:

- ```calibration/resource/calibration.ui``` – the actual interface
- ```calibration/src/rqt_calibration_gui/calibrationGUI.py``` – functionality of the interface
- ```calibration/src/config.json``` – configuration file for testbed, robot arm, aruco and checkerboard options

```calibrationGUI.py``` is where all the components of the project come together. This contains the code that calls the camera calibration, arm calibration, and RViz visualization. 

### Making Changes
When making changes to the GUI, use [QtDesigner](https://build-system.fman.io/qt-designer-download)  to add, update, or delete GUI elements in ```calibration.ui```. 

The names in the object inspector (top right) are editable and correspond to how you access these elements in ```calibrationGUI.py```.

- **Ex.** to access the element with the name *pictureButton* in ```CalibrationGUI.py``` use ```self._widget.pictureButton```

- **Ex.** To add an event handler on click, refer to the following code:
<code>self._widget.pictureButton.clicked[bool].connect(self.handle_take_picture_clicked)</code>

To update when widgets are grayed out, refer to the function ```self.update_enabled()```. This function enables/disables widgets based on the state of the GUI i.e. different variables set to true/false. Make sure to call this function after an event sets an important boolean variable.

For more information about widgets, widget properties, widget functions, etc… Refer to the qt documentation for python 2 here:
https://doc.qt.io/qtforpython-5/PySide2/QtWidgets/index.html 


<h2>Camera Calibration</h2>

### Important Files:

- ```calibration/src/camera_calibration.py``` – functionality of the camera calibration in test environment 
    
    <br> ```self.camera_1_callback(msg)``` - this function will save a photo that has been taken with the testbed camera
    <br> ```self.camera_calibration_srv_callback(request)``` - this function will perform the respective camera calibration functions. Function has the option to overwrite values given such as height, distortion, camera matrix, and configuration values.
    <br> ```self.calibrate_internal()``` - this function will calibrate the camera's internal values, and generates the distortion vector and camera matrix. The function requires thirty photos of the checkerboard to have been taken and saved with the camera
    <br> ```self.calibrate_height(distortion, camera_matrix)``` - this function will calculate the distance of the camera to the table and returns that value in meters. The function requires a photo of the checkerboard laying flat on the test environment. Utilizes the distortion vector and camera matrix to account for intrinsic properties of camera.
    <br> ```self.calibrate_arucos(distortion, camera_matrix, height)``` - this function will calculate and return the transformation matrix from the table to the camera. The function requires a photo of the checkerboard laying flat on the test environment with the four AruCos visible in the corner of the test environment. The translation component of the matrix is calculated by averaging the x and y components of each AruCo translation, and using the height calculated earlier for the z component. The translation component of the matrix is calculated by generating x and y vectors that intersect at the center of the table and generating an axis at the intersection. This will create the transformation from the camera to table, which needs to be inverted to get the table to camera matrix. Utilizes the distortion vector and camera matrix to account for intrinsic properties of camera.<br><br>
    
- ```calibration/src/config.json``` – configuration file that specifies meta information, including aruco and checkerboard options

    <br>```checkerboard_rows_default``` defines the number of rows in the checkerboard paper used for camera calibration. The default is `6`
    <br>```checkerboard_cols_default``` defines the number of columns in the checkerboard paper used for camera calibration. The default is `9`
    <br>```aruco_sidelength``` defines the length of the side of a single AruCo square. The default is `0.036`
    <br>```aruco_dict_used``` defines which AruCo dictionary is being used. Potential options are: `DICT_4X4_50`, `DICT_4X4_100`, `DICT_4X4_250`, `DICT_4X4_1000`, `DICT_5X5_50`, `DICT_5X5_100`, `DICT_5X5_250`, `DICT_5X5_1000`, `DICT_6X6_50`, `DICT_6X6_100`, `DICT_6X6_250`, `DICT_6X6_1000`, `DICT_7X7_50`, `DICT_7X7_100`, `DICT_7X7_250`, `DICT_7X7_1000`, `DICT_ARUCO_ORIGINAL`. The default is `DICT_5X5_1000`.
    <br>```aruco_ids```: defines the AruCo IDs of each of the corners of the test environment. The order of the array should be [Top Left ID, Top Right ID, Bottom Left ID, Bottom Right ID]<br><br>
    
### Making Changes

For more information about the camera calibration, refer to OpenCV documentation. Some useful links include [Internal Camera Calibration Tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html), [Pose Estimation Tutorial](https://docs.opencv.org/4.x/d7/d53/tutorial_py_pose.html), [AruCo Pose Estimation](https://docs.opencv.org/4.x/d9/d6a/group__aruco.html#ga84dd2e88f3e8c3255eb78e0f79571bd1)


<h2>Arm Calibration</h2>

### Important Files:
- ```calibration/src/arm_calibration.py``` – functionality to calibrate the robot arm in the test environment
    <br>
    <br> ```self.calibrate_arm(tp_data)``` - This function returns a calibrated transformation matrix from the center of the testbed to the robot arm base.
    <br> ```self.__getTranslations(tp_data)``` - Returns 4 arrays (Based off the 4 touchpoints) containing the XYZ coordinates of the end effector positions in the world frame.
    <br> ```self.__calibrateTranslations(arr_translations)``` - Calibrates the XYZ location of the center of the testbed from the robot arm base.
    <br> ```self.__calibrateRotations(arr_translations)``` - Calibrates the rotation portion of the transformation matrix.
    <br> ```self.__updateTransMatrix(arr_translations)``` - Creates a calibrated transformation matrix from the robot arm base to the center of the table.
    <br> ```self.__recordTouchpoint(id)``` - Records the end effector location in the world frame. Stores transformation matrix in tp_data.
    <br> ```self.record_touchpoint_srv_callback(request)``` - The callback used by the GUI to collect the touchpoint data for a specific touchpoint id.
    <br> ```self.arm_calibration_srv_callback(request)``` - The callback used by the GUI to calculate the final transform matrix.
### Making Changes
Editing this file may require the developer to have an understanding of numpy and tf. Documentation can be found here:
<br>https://numpy.org/doc/stable/user/index.html#user
<br>http://wiki.ros.org/tf

<h2>Visulization</h2>

### Important Files:
- ```calibration/src/visualization.py``` – Publishes tf frames and markers for the testbed and camera in relation to the arm frame.
    <br>
    <br> ```self.display_result_srv_callback(self, request)``` - SRV callback that triggers us to publish tf frames and markers. request passes in the arm_to_table matrix and the table_to_camera matrix
    <br> ```self.pub_testbed_frame()``` - Publishes the tf frame for the testbed in relation to the arm base frame.
    <br> ```self.pub_camera_frame()``` - Publishes the tf frame for the camera in relation to the testbed frame.
    <br> ```self.pub_testbed_marker() ``` - Publishes the testbed marker at the origin of the testbed frame
    <br> ```self.pub_camera_marker() ``` - Publishes the camera marker at the origin of the camera frame
    <br> ```self.show_result()``` - Calls above four function to continually publish tf frames and markers..
    
### Making Changes
Editing this file may require the developer to have an understanding of numpy, tf, ROS and Rviz. Documentation can be found here:
<br>https://numpy.org/doc/stable/user/index.html#user
<br>http://wiki.ros.org/tf


<h2>Unrealized Features</h2>
 
 - Multiple camera support (Cameras in multiple positions)
 
 - Non planar testbeds 

