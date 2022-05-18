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
