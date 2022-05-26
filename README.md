[GitHub Repo Link](https://github.com/Keeganfn/testbed-calibration)

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
