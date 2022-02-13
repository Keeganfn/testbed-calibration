import os
import rospy
import actionlib
import rospkg

from argparse import ArgumentParser
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtGui import QBrush, QColor, QIcon
from python_qt_binding.QtWidgets import QWidget, QAbstractItemView, QTableWidgetItem, QAbstractScrollArea, QMessageBox, QApplication

class CalibrationGUI(Plugin):

  def __init__(self, context):
    '''init initializes the widget and sets up our subscriber, publisher and event handlers'''
    super(CalibrationGUI, self).__init__(context)
    self.setObjectName('CalibrationGUI')

    # Process standalone plugin command-line arguments
    parser = ArgumentParser()
    parser.add_argument("-q", "--quiet", action="store_true", dest="quiet", help="Put plugin in silent mode")
    args, unknowns = parser.parse_known_args(context.argv())

    # Find resources and Create QWidget
    self._widget = QWidget()
    ui_file = os.path.join(rospkg.RosPack().get_path('calibration'), 'resource', 'calibration.ui')
    loadUi(ui_file, self._widget)
    self._widget.setObjectName('CalibrationUI')

    if context.serial_number() > 1:
      self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
    context.add_widget(self._widget)

    # Setup button event listeners
    self._widget.importButton.clicked[bool].connect(self.handle_import_clicked)
    self._widget.pictureButton.clicked[bool].connect(self.handle_take_picture_clicked)
    self._widget.cameraPoseButton.clicked[bool].connect(self.handle_calibrate_camera_pose_clicked)
    self._widget.upperLeftButton.clicked[bool].connect(self.handle_upper_left_touchpt_clicked)
    self._widget.upperRightButton.clicked[bool].connect(self.handle_upper_right_touchpt_clicked)
    self._widget.lowerLeftButton.clicked[bool].connect(self.handle_lower_left_touchpt_clicked)
    self._widget.lowerRightButton.clicked[bool].connect(self.handle_lower_right_touchpt_clicked)
    self._widget.saveCamSettingsButton.clicked[bool].connect(self.handle_save_camera_settings_clicked)
    self._widget.viewTButton.clicked[bool].connect(self.handle_view_top_clicked)
    self._widget.viewSButton.clicked[bool].connect(self.handle_view_side_clicked)
    self._widget.saveAsButton.clicked[bool].connect(self.handle_save_as_clicked)
    self._widget.startRVizButton.clicked[bool].connect(self.handle_start_RViz_clicked)

    # Setup combo box event listeners
    self._widget.testBedComboBox.currentIndexChanged.connect(self.handle_testbed_change)
    self._widget.armComboBox.currentIndexChanged.connect(self.handle_arm_change)
    self._widget.cameraComboBoxT.currentIndexChanged.connect(self.handle_camera_T_change)
    self._widget.cameraComboBoxS.currentIndexChanged.connect(self.handle_camera_S_change)

    # Get zdist value manually 
    #self._widget.zDistSpinBox.value()

    #Inital guess values
    #self._widget.xSpinBox.value()
    #self._widget.ySpinBox.value()
    #self._widget.zSpinBox.value()

  
  def handle_import_clicked(self):
    print("importing internal camera calibration settings")

  def handle_take_picture_clicked(self):
    print("taking picture")

  def handle_calibrate_camera_pose_clicked(self):
    # Set label content
    # self._widget.zDistFoundLabel.setText(self._widget.zDistFoundLabel.text() + "0")
    print("calibrating camera pose")

  def handle_testbed_change(self, i):
    print("Testbed index is:", i, "Value:", self._widget.testBedComboBox.currentText())

  def handle_arm_change(self, i):
    print("Arm index is:", i, "Value:", self._widget.armComboBox.currentText())

  def handle_camera_T_change(self, i):
    print("Camera index is:", i, "Value:", self._widget.cameraComboBoxT.currentText())

  def handle_camera_S_change(self, i):
    print("Camera index is:", i, "Value:", self._widget.cameraComboBoxS.currentText())

  def handle_upper_left_touchpt_clicked(self):
    # Set label content
    # self._widget.upperLeftLabel.setText(self._widget.upperLeftLabel.text() + " Done")
    print("getting upper left touchpt")

  def handle_upper_right_touchpt_clicked(self):
    print("getting upper right touchpt")

  def handle_lower_left_touchpt_clicked(self):
    print("getting lower left touchpt")

  def handle_lower_right_touchpt_clicked(self):
    print("getting lower right touchpt")
  
  def handle_save_camera_settings_clicked(self):
    print("saving camera settings")
  
  def handle_view_top_clicked(self):
    print("viewing top camera")
  
  def handle_view_side_clicked(self):
    print("viewing side camera")
  
  def handle_save_as_clicked(self):
    print("saving settings")
  
  def handle_start_RViz_clicked(self):
    print("starting RViz")
