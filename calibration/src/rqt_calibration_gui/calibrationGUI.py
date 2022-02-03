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
    self._widget.calibrateInternalCameraButton.clicked[bool].connect(self.handle_internal_camera_calibration_clicked)
    self._widget.cameraPoseButton.clicked[bool].connect(self.handle_calibrate_camera_pose_clicked)
    self._widget.recordButton.clicked[bool].connect(self.handle_record_touch_pt_clicked)

    # Setup combo box event listeners
    self._widget.testBedComboBox.currentIndexChanged.connect(self.handle_testbed_change)
    self._widget.armComboBox.currentIndexChanged.connect(self.handle_arm_change)
    self._widget.cameraComboBox.currentIndexChanged.connect(self.handle_camera_change)
  
  def handle_import_clicked(self):
    print("importing internal camera calibration settings")

  def handle_internal_camera_calibration_clicked(self):
    print("calibrating internal camera")

  def handle_calibrate_camera_pose_clicked(self):
    print("calibrating camera pose")

  def handle_record_touch_pt_clicked(self):
    print("recording touchpoint")

  def handle_testbed_change(self, i):
      print("Testbed index is:", i, "Value:", self._widget.testBedComboBox.currentText())

  def handle_arm_change(self, i):
      print("Arm index is:", i, "Value:", self._widget.armComboBox.currentText())

  def handle_camera_change(self, i):
      print("Camera index is:", i, "Value:", self._widget.cameraComboBox.currentText())

