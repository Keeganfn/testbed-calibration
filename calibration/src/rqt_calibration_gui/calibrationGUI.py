import os
import rospy
import actionlib
import rospkg

from argparse import ArgumentParser
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtGui import QBrush, QColor, QIcon
from python_qt_binding.QtWidgets import QWidget, QAbstractItemView, QTableWidgetItem, QAbstractScrollArea, QMessageBox, QApplication, QDialog, QFileDialog

from std_srvs.srv import Trigger, TriggerResponse
from calibration.srv import CameraCalibrationSRV, CameraCalibrationSRVResponse, CameraCalibrationSRVRequest
from calibration.srv import ArmCalibrationSRV, ArmCalibrationSRVResponse, ArmCalibrationSRVRequest
from calibration.srv import ArmRecordPointSRV, ArmRecordPointSRVResponse, ArmRecordPointSRVRequest

import csv

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

        rospy.wait_for_service("take_picture_srv")
        self.take_picture_client = rospy.ServiceProxy("take_picture_srv", Trigger)
        rospy.wait_for_service("camera_calibration_srv")
        self.camera_calibration_client = rospy.ServiceProxy("camera_calibration_srv", CameraCalibrationSRV)
        rospy.wait_for_service("arm_calibration_srv")
        self.arm_calibration_client = rospy.ServiceProxy("arm_calibration_srv", ArmCalibrationSRV)
        rospy.wait_for_service("record_touchpoint_srv")
        self.record_touchpoint_client = rospy.ServiceProxy("record_touchpoint_srv", ArmRecordPointSRV)

        self.testbed_selected = None
        self.arm_selected = None
        self.camera_selected = None
        self.camera_distortion = None
        self.camera_matrix = None
        self.camera_transform_matrix = None
        self.arm_transform_matrix = None

        self.total_pictures = 0
        self.z_dist = 0

        # Setup button event listeners
        self._widget.importCalibButton.clicked[bool].connect(self.handle_import_calib_clicked)
        self._widget.importCamButton.clicked[bool].connect(self.handle_import_cam_clicked)
        self._widget.pictureButton.clicked[bool].connect(self.handle_take_picture_clicked)
        self._widget.cameraPoseButton.clicked[bool].connect(self.handle_calibrate_camera_pose_clicked)
        self._widget.markCompleteButton.clicked[bool].connect(self.handle_mark_complete_clicked)
        self._widget.upperLeftButton.clicked[bool].connect(self.handle_upper_left_touchpt_clicked)
        self._widget.upperRightButton.clicked[bool].connect(self.handle_upper_right_touchpt_clicked)
        self._widget.lowerLeftButton.clicked[bool].connect(self.handle_lower_left_touchpt_clicked)
        self._widget.lowerRightButton.clicked[bool].connect(self.handle_lower_right_touchpt_clicked)
        self._widget.saveCalibButton.clicked[bool].connect(self.handle_save_camera_calib_clicked)
        self._widget.saveAsButton.clicked[bool].connect(self.handle_save_as_clicked)
        self._widget.startRVizButton.clicked[bool].connect(self.handle_start_RViz_clicked)

        # Setup combo box event listeners
        self._widget.testBedComboBox.currentIndexChanged.connect(self.handle_testbed_change)
        self._widget.armComboBox.currentIndexChanged.connect(self.handle_arm_change)

        # Table
        self._widget.camTable.itemClicked.connect(self.handle_cam_table_item_clicked)
        

        # Determining when to enable things
        self.is_testbed_selected = False
        self.is_arm_selected = False
        self.is_camera_selected = False
        self.is_touchpt_recording_done = [False, False, False, False]
        self.is_internal_camera_calibrated = False
        self.is_camera_pose_calibrated = False
        self.is_camera_calibration_imported = False
        self.is_mark_complete = False
        self.is_import_calib_clicked = False

        # Grey out everything
        self._widget.pictureButton.setEnabled(False)
        self._widget.camTable.setEnabled(False)
        self._widget.importCamButton.setEnabled(False)
        self._widget.cameraPoseButton.setEnabled(False)
        self._widget.zDistSpinBox.setEnabled(False)
        self._widget.xSpinBox.setEnabled(False)
        self._widget.ySpinBox.setEnabled(False)
        self._widget.zSpinBox.setEnabled(False)
        self._widget.markCompleteButton.setEnabled(False)
        self._widget.upperLeftButton.setEnabled(False)
        self._widget.upperRightButton.setEnabled(False)
        self._widget.lowerLeftButton.setEnabled(False)
        self._widget.lowerRightButton.setEnabled(False)
        self._widget.saveCalibButton.setEnabled(False)
        self._widget.saveAsButton.setEnabled(False)
        self._widget.startRVizButton.setEnabled(False)


        # Get zdist value manually 
        #self._widget.zDistSpinBox.value()

        #Inital guess values
        #self._widget.xSpinBox.value()
        #self._widget.ySpinBox.value()
        #self._widget.zSpinBox.value()
        
        # Testing adding in row to camera table
        self.insert_camera_to_table(["Camera 1", "No"])

    def read_camera_calibration_csv(self, filename):
        with open(filename, "r") as f:
            csv_reader = csv.reader(f, delimiter = ",", lineterminator="\n")
            csv_arr = []
            for row in csv_reader:
                csv_arr.append(row)

            self.testbed_selected = csv_arr[1][0]
            self.arm_selected = csv_arr[1][1]
            self.camera_selected = csv_arr[1][2]
            self.camera_distortion = csv_arr[1][3]
            self.camera_matrix = csv_arr[1][4]
            self.camera_transform_matrix = csv_arr[1][5]

    def read_calibration_csv(self, filename):
        with open(filename, "r") as f:
            csv_reader = csv.reader(f, delimiter = ",", lineterminator="\n")
            csv_arr = []
            for row in csv_reader:
                csv_arr.append(row)

            self.testbed_selected = csv_arr[1][0]
            self.arm_selected = csv_arr[1][1]
            self.camera_selected = csv_arr[1][2]
            self.camera_distortion = csv_arr[1][3]
            self.camera_matrix = csv_arr[1][4]
            self.camera_transform_matrix = csv_arr[1][5]
            self.arm_transform_matrix = csv_arr[1][6]

    def save_calibration_to_csv(self, filename):
        filename = filename + ".csv"
        with open(filename,"w") as f:
            csv_writer = csv.writer(f,delimiter=",",lineterminator="\n")
            csv_writer.writerow(["testbed_selected", "arm_selected", "camera_selected", "camera_distortion","camera_matrix","camera_transform_matrix", "arm_transform_matrix"])
            csv_writer.writerow([self.testbed_selected, self.arm_selected, self.camera_selected, self.camera_distortion,self.camera_matrix,self.camera_transform_matrix, self.arm_transform_matrix])

    def save_camera_calibration_to_csv(self, filename):
        filename = filename + ".csv"
        with open(filename,"w") as f:
            csv_writer = csv.writer(f,delimiter=",",lineterminator="\n")
            csv_writer.writerow(["testbed_selected", "arm_selected", "camera_selected", "camera_distortion","camera_matrix","camera_transform_matrix"])
            csv_writer.writerow([self.testbed_selected, self.arm_selected, self.camera_selected, self.camera_distortion,self.camera_matrix,self.camera_transform_matrix])

    def insert_camera_to_table(self, row_list):
            row_count = self._widget.camTable.rowCount()
            self._widget.camTable.setRowCount(row_count+1)

            curr_col = 0
            for value in row_list:
                item = QTableWidgetItem(value)
                self._widget.camTable.setItem(row_count, curr_col, item)
                curr_col += 1

    def update_enabled(self):
        self._widget.camTable.setEnabled(self.is_testbed_selected and self.is_arm_selected and (not self.is_import_calib_clicked))
        self._widget.importCamButton.setEnabled(self.is_testbed_selected and self.is_arm_selected)

        self._widget.pictureButton.setEnabled(self.is_camera_selected and (not self.is_camera_calibration_imported) and (not self.is_internal_camera_calibrated) and (not self.is_import_calib_clicked))

        camera_pose_enabled = self.is_internal_camera_calibrated and (not self.is_camera_calibration_imported) and (not self.is_mark_complete) and (not self.is_import_calib_clicked)
        self._widget.cameraPoseButton.setEnabled(camera_pose_enabled)
        self._widget.zDistSpinBox.setEnabled(camera_pose_enabled)
        self._widget.markCompleteButton.setEnabled(camera_pose_enabled and self.is_camera_pose_calibrated)
        self._widget.saveCalibButton.setEnabled(camera_pose_enabled)

        touchpt_section_enabled = self.is_camera_pose_calibrated and self.is_mark_complete and (not self.is_import_calib_clicked)
        self._widget.xSpinBox.setEnabled(touchpt_section_enabled)
        self._widget.ySpinBox.setEnabled(touchpt_section_enabled)
        self._widget.zSpinBox.setEnabled(touchpt_section_enabled)
        self._widget.upperLeftButton.setEnabled(touchpt_section_enabled)
        self._widget.upperRightButton.setEnabled(touchpt_section_enabled)
        self._widget.lowerLeftButton.setEnabled(touchpt_section_enabled)
        self._widget.lowerRightButton.setEnabled(touchpt_section_enabled)

        self._widget.saveAsButton.setEnabled(all(self.is_touchpt_recording_done))
        self._widget.startRVizButton.setEnabled(all(self.is_touchpt_recording_done) or self.is_import_calib_clicked)

        self._widget.testBedComboBox.setEnabled(not self.is_import_calib_clicked)
        self._widget.armComboBox.setEnabled(not self.is_import_calib_clicked)
    
    def handle_import_calib_clicked(self):
        print("importing calibration settings")
        # Get calibration file
        filename, selecter_filter = QFileDialog.getOpenFileName(self._widget, "Import Calibration", "/home", "CSV files (*.csv)")

        if filename != None and filename != "":
            self.read_calibration_csv(filename)
            self.is_import_calib_clicked = True

        self.update_enabled()

    def handle_import_cam_clicked(self):
        print("importing camera settings")
        # Get camera calibration file
        filename, selected_filter = QFileDialog.getOpenFileName(self._widget, "Import Camera Calibration", "/home", "CSV files (*.csv)")

        if filename != None and filename != "":
            self.read_camera_calibration_csv(filename)
            self.is_camera_calibration_imported = True
            self.is_internal_camera_calibrated = True
            self.is_camera_pose_calibrated = True
            self.is_mark_complete = True

        self.update_enabled()

    def handle_cam_table_item_clicked(self, item):
        # 0 is camera column, 1 is status column
        if item.column() == 0:
            self._widget.currentCamSelectionLabel.setText("Current Selection: " + item.text())
            if item.text() != "None":
                self.camera_selected = item.text()
                self.is_camera_selected = True
            else:
                self.is_camera_selected = False

        self.update_enabled()


    def handle_take_picture_clicked(self):
        print("taking picture")

        #Requests service to take picture
        response = TriggerResponse()
        response = self.take_picture_client()
        rospy.loginfo("PICTURE TAKEN: {0}".format(response.success))
        
        #Update progress bar
        if self.total_pictures < 30:
            self.total_pictures+=1
            self._widget.pictureProgressLabel.setText(str(self.total_pictures)+"/30")
            self._widget.progressBar.setValue(self.total_pictures)

            if self.total_pictures == 30:
                self.is_internal_camera_calibrated = True

        self.update_enabled()

    def handle_calibrate_camera_pose_clicked(self):
        # Set label content
        print("calibrating camera pose")
        response = CameraCalibrationSRVResponse()
        response = self.camera_calibration_client(self.z_dist)
        rospy.loginfo("FOUND {0} {1} {2}".format(response.distortion, response.camera_matrix, response.transform_matrix))

        self.camera_distortion = response.distortion
        self.camera_matrix = response.camera_matrix
        self.camera_transform_matrix = response.transform_matrix

        self._widget.zDistFoundLabel.setText("Z Distance Found: "+ str(self.z_dist))

        self.is_camera_pose_calibrated = True
        self.update_enabled()

    def record_touchpoint(self,location):
        response = ArmRecordPointSRVResponse()
        response = self.record_touchpoint_client(location)
        rospy.loginfo("TOUCHPOINT RECORD: {0}".format(response.success))

    def handle_testbed_change(self, i):
        if i != 0:
            self.is_testbed_selected = True
            self.testbed_selected = self._widget.testBedComboBox.currentText()
        else:
            self.is_testbed_selected = False
        print("Testbed index is:", i, "Value:", self._widget.testBedComboBox.currentText())
        self.update_enabled()

    def handle_arm_change(self, i):
        if i != 0:
            self.is_arm_selected = True
            self.arm_selected = self._widget.armComboBox.currentText()
        else:
            self.is_arm_selected = False
        print("Arm index is:", i, "Value:", self._widget.armComboBox.currentText())
        self.update_enabled()

    def handle_upper_left_touchpt_clicked(self):
        # Set label content
        self.record_touchpoint(1)
        self._widget.upperLeftLabel.setText("Status: Done")
        print("getting upper left touchpt")

        self.is_touchpt_recording_done[1] = True
        self.update_enabled()

    def handle_upper_right_touchpt_clicked(self):
        self.record_touchpoint(2)
        self._widget.upperRightLabel.setText("Status: Done")
        print("getting upper right touchpt")

        self.is_touchpt_recording_done[2] = True
        self.update_enabled()

    def handle_lower_left_touchpt_clicked(self):
        self.record_touchpoint(0)
        self._widget.lowerLeftLabel.setText("Status: Done")
        print("getting lower left touchpt")
        self.is_touchpt_recording_done[0] = True
        self.update_enabled()

    def handle_lower_right_touchpt_clicked(self):
        self.record_touchpoint(3)
        self._widget.lowerRightLabel.setText("Status: Done")
        print("getting lower right touchpt")
        self.is_touchpt_recording_done[3] = True
        self.update_enabled()
    
    def handle_mark_complete_clicked(self):
        print("marking complete")
        self.is_mark_complete = True
        self.update_enabled()
    
    def handle_save_camera_calib_clicked(self):
        print("saving camera calib settings")
        filename, selected_filter = QFileDialog.getSaveFileName(self._widget, "Save Camera Calibration", "/home", "CSV files (*.csv)")

        if filename != None and filename != "":
            self.save_calibration_to_csv(filename)

        self.update_enabled()
    
    def handle_save_as_clicked(self):
        print("saving settings")
        response = ArmCalibrationSRV()
        response = self.arm_calibration_client(1,2,3)
        rospy.loginfo("FOUND: {0}".format(response.transform_matrix))

        self.arm_transform_matrix = response.transform_matrix

        filename, selected_filter = QFileDialog.getSaveFileName(self._widget, "Save Calibration", "/home", "CSV files (*.csv)")

        if filename != None and filename != "":
            self.save_calibration_to_csv(filename)
    
    def handle_start_RViz_clicked(self):
        print("starting RViz")
        print(self.testbed_selected)
        print(self.arm_selected)
        print(self.camera_selected)
        print(self.camera_distortion)
        print(self.camera_matrix)
        print(self.camera_transform_matrix)
        print(self.arm_transform_matrix)
