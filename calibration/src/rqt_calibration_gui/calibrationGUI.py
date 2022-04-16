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
from calibration.srv import DisplayResultSRV, DisplayResultSRVResponse, DisplayResultSRVRequest

import csv
from ast import literal_eval

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

        #NEED TO HAVE MORE ELEGANT WAIT, CRASHES RVIZ
        rospy.wait_for_service("take_picture_srv")
        self.take_picture_client = rospy.ServiceProxy("take_picture_srv", Trigger)
        rospy.wait_for_service("display_result_srv")
        self.display_result_client = rospy.ServiceProxy("display_result_srv", DisplayResultSRV)
        rospy.wait_for_service("camera_calibration_srv")
        self.camera_calibration_client = rospy.ServiceProxy("camera_calibration_srv", CameraCalibrationSRV)
        rospy.wait_for_service("arm_calibration_srv")
        self.arm_calibration_client = rospy.ServiceProxy("arm_calibration_srv", ArmCalibrationSRV)
        rospy.wait_for_service("record_touchpoint_srv")
        self.record_touchpoint_client = rospy.ServiceProxy("record_touchpoint_srv", ArmRecordPointSRV)

        # Stored values of calibration
        self.testbed_selected = None
        self.arm_selected = None
        self.camera_selected = None
        self.camera_distortion = None
        self.camera_matrix = None
        self.camera_matrix_step = None
        self.camera_transform_matrix = None
        self.camera_transform_matrix_step = None
        self.arm_transform_matrix = None
        self.arm_transform_matrix_step = None
        self.z_dist = 0
        self.arm_initial_guess = [0,0,0]

        self.total_pictures = 0

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

        # Z-dist changed manually
        self._widget.zDistSpinBox.valueChanged.connect(self.handle_z_spinbox_change)

        # Initial guesses for arm
        self._widget.xSpinBox.valueChanged.connect(self.handle_initial_guess_x_spinbox_change)
        self._widget.ySpinBox.valueChanged.connect(self.handle_initial_guess_y_spinbox_change)
        self._widget.zSpinBox.valueChanged.connect(self.handle_initial_guess_z_spinbox_change)


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

        # Csv headers
        self.csv_calibration_header = ["testbed_selected", "arm_selected", "camera_selected", 
                             "camera_distortion", "camera_matrix_step", "camera_matrix",
                             "camera_transform_matrix_step", "camera_transform_matrix", 
                             "arm_transform_matrix_step", "arm_transform_matrix"]

        self.csv_camera_header = ["testbed_selected", "arm_selected", "camera_selected", 
                             "camera_distortion", "camera_matrix_step", "camera_matrix",
                             ] 

        # Testing adding in row to camera table
        self.insert_camera_to_table(["Camera 1", "No"])

    # Updating z dist manually
    def update_z_dist_in_camera_matrices(self, new_z):
        camera_matrix_list = list(self.camera_matrix)
        camera_transform_matrix_list = list(self.camera_transform_matrix)

        if self.camera_matrix_step == 3:
            camera_matrix_list[-1] = new_z
        elif self.camera_matrix_step == 4:
            camera_matrix_list[-5] = new_z

        if self.camera_transform_matrix_step == 3:
            camera_transform_matrix_list[-1] = new_z
        elif self.camera_transform_matrix_step == 4:
            camera_transform_matrix_list[-5] = new_z

        self.camera_matrix = tuple(camera_matrix_list)
        self.camera_transform_matrix = tuple(camera_transform_matrix_list)

    # Dialog for import / save buttons on failure
    def create_error_dialog(self, error_msg):
        error_dialog = QMessageBox(self._widget)
        error_dialog.setIcon(QMessageBox.Critical)
        error_dialog.setText("Error")
        error_dialog.setInformativeText(error_msg)
        error_dialog.setWindowTitle("Error")
        error_dialog.exec_()

    def read_camera_calibration_csv(self, filename):
        with open(filename, "r") as f:
            csv_reader = csv.reader(f, delimiter = ",", lineterminator="\n")
            csv_arr = []
            for row in csv_reader:
                csv_arr.append(row)

            # Make sure csv column headers are the same
            for idx, col_header in enumerate(csv_arr[0]):
                if col_header != self.csv_camera_header[idx]:
                    raise Exception

            self.testbed_selected = csv_arr[1][0]
            self.arm_selected = csv_arr[1][1]
            self.camera_selected = csv_arr[1][2]
            self.camera_distortion = literal_eval(csv_arr[1][3])
            self.camera_matrix_step = literal_eval(csv_arr[1][4])
            self.camera_matrix = literal_eval(csv_arr[1][5])

    def read_calibration_csv(self, filename):
        with open(filename, "r") as f:
            csv_reader = csv.reader(f, delimiter = ",", lineterminator="\n")
            csv_arr = []
            for row in csv_reader:
                csv_arr.append(row)

            # Make sure csv column headers are the same
            for idx, col_header in enumerate(csv_arr[0]):
                if col_header != self.csv_calibration_header[idx]:
                    raise Exception

            self.testbed_selected = csv_arr[1][0]
            self.arm_selected = csv_arr[1][1]
            self.camera_selected = csv_arr[1][2]
            self.camera_distortion = literal_eval(csv_arr[1][3])
            self.camera_matrix_step = literal_eval(csv_arr[1][4])
            self.camera_matrix = literal_eval(csv_arr[1][5])
            self.camera_transform_matrix_step = literal_eval(csv_arr[1][6])
            self.camera_transform_matrix = literal_eval(csv_arr[1][7])
            self.arm_transform_matrix_step = literal_eval(csv_arr[1][8])
            self.arm_transform_matrix = literal_eval(csv_arr[1][9])

    def save_calibration_to_csv(self, filename):
        filename = filename + ".csv"
        with open(filename,"w") as f:
            csv_writer = csv.writer(f,delimiter=",",lineterminator="\n")
            csv_writer.writerow(self.csv_calibration_header)
            csv_writer.writerow([self.testbed_selected, self.arm_selected, self.camera_selected,
                                 self.camera_distortion, self.camera_matrix_step, self.camera_matrix,
                                 self.camera_transform_matrix_step, self.camera_transform_matrix, 
                                 self.arm_transform_matrix_step, self.arm_transform_matrix])

    def save_camera_calibration_to_csv(self, filename):
        filename = filename + ".csv"
        with open(filename,"w") as f:
            csv_writer = csv.writer(f,delimiter=",",lineterminator="\n")
            csv_writer.writerow(self.csv_camera_header)
            csv_writer.writerow([self.testbed_selected, self.arm_selected, self.camera_selected, 
                                 self.camera_distortion, self.camera_matrix_step, self.camera_matrix,
                                 ])

    # Adding cameras into the table
    def insert_camera_to_table(self, row_list):
            row_count = self._widget.camTable.rowCount()
            self._widget.camTable.setRowCount(row_count+1)

            curr_col = 0
            for value in row_list:
                item = QTableWidgetItem(value)
                self._widget.camTable.setItem(row_count, curr_col, item)
                curr_col += 1

    # Updates what is greyed out
    def update_enabled(self):
        self._widget.camTable.setEnabled(self.is_testbed_selected and self.is_arm_selected and (not self.is_import_calib_clicked))
        self._widget.importCamButton.setEnabled(self.is_testbed_selected and self.is_arm_selected)

        self._widget.pictureButton.setEnabled(self.is_camera_selected and (not self.is_camera_calibration_imported) and (not self.is_internal_camera_calibrated) and (not self.is_import_calib_clicked))

        camera_pose_enabled = self.is_internal_camera_calibrated and (not self.is_mark_complete) and (not self.is_import_calib_clicked)
        self._widget.cameraPoseButton.setEnabled(camera_pose_enabled)
        self._widget.zDistSpinBox.setEnabled(camera_pose_enabled and self.is_camera_pose_calibrated)
        self._widget.markCompleteButton.setEnabled(camera_pose_enabled and self.is_camera_pose_calibrated)
        self._widget.saveCalibButton.setEnabled(camera_pose_enabled and self.is_camera_pose_calibrated)

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
    

    def record_touchpoint(self,location):
        response = ArmRecordPointSRVResponse()
        response = self.record_touchpoint_client(location)
        rospy.loginfo("TOUCHPOINT RECORD: {0}".format(response.success))

    ####################################################################################
    ### GUI Event handlers
    ####################################################################################

    def handle_import_calib_clicked(self):
        print("importing calibration settings")

        try:
            # Get calibration file
            filename, selecter_filter = QFileDialog.getOpenFileName(self._widget, "Import Calibration", "/home", "CSV files (*.csv)")

            if filename != None and filename != "":
                self.read_calibration_csv(filename)
                self.is_import_calib_clicked = True
        except Exception as ex:
            self.create_error_dialog("Failed to import " + filename + ". Check the file format." + str(ex))

            # Reset values in case they were set when reading garbage csv
            self.testbed_selected = None
            self.arm_selected = None
            self.camera_selected = None
            self.camera_distortion = None
            self.camera_matrix = None
            self.camera_matrix_step = None
            self.camera_transform_matrix = None
            self.camera_transform_matrix_step = None
            self.arm_transform_matrix = None
            self.arm_transform_matrix_step = None

        self.update_enabled()


    def handle_import_cam_clicked(self):
        print("importing camera settings")
        # Get camera calibration file
        filename, selected_filter = QFileDialog.getOpenFileName(self._widget, "Import Camera Calibration", "/home", "CSV files (*.csv)")

        try:
            if filename != None and filename != "":
                self.read_camera_calibration_csv(filename)
                self.is_camera_calibration_imported = True
                self.is_internal_camera_calibrated = True

        except Exception as ex:
            self.create_error_dialog("Failed to import " + filename + ". Check the file format." + str(ex))

            # Reset values in case they were set when reading garbage csv
            self.testbed_selected = None
            self.arm_selected = None
            self.camera_selected = None
            self.camera_distortion = None
            self.camera_matrix_step = None
            self.camera_matrix = None

            self.is_testbed_selected = False
            self.is_arm_selected = False
            self.is_camera_selected = False

            self._widget.testBedComboBox.setCurrentIndex(0)
            self._widget.armComboBox.setCurrentIndex(0)

            self._widget.currentCamSelectionLabel.setText("Current Selection: None")

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
        response = TriggerResponse()
        response = self.take_picture_client()
        rospy.loginfo("PICTURE TAKEN: {0}".format(response.success))
        
        response = CameraCalibrationSRVResponse()
        response = self.camera_calibration_client(self.is_camera_calibration_imported, self.camera_distortion, 
                                                  self.camera_matrix_step, self.camera_matrix)
        rospy.loginfo("FOUND {0} {1} {2}".format(response.distortion, response.camera_matrix, response.transform_matrix))
        self.camera_distortion = response.distortion
        self.camera_matrix = response.camera_matrix
        self.camera_matrix_step = response.camera_matrix_step
        self.camera_transform_matrix = response.transform_matrix
        self.camera_transform_matrix_step = response.transform_matrix_step

        self._widget.zDistFoundLabel.setText("Z Distance Found: "+ str(self.z_dist))

        self.is_camera_pose_calibrated = True
        self.update_enabled()

    def handle_z_spinbox_change(self, new_value):
        self.z_dist = new_value

        self.update_z_dist_in_camera_matrices(self.z_dist)

    def handle_initial_guess_x_spinbox_change(self, new_value):
        self.arm_initial_guess[0] = new_value

    def handle_initial_guess_y_spinbox_change(self, new_value):
        self.arm_initial_guess[1] = new_value

    def handle_initial_guess_z_spinbox_change(self, new_value):
        self.arm_initial_guess[2] = new_value

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
        try:
            filename, selected_filter = QFileDialog.getSaveFileName(self._widget, "Save Camera Calibration", "/home", "CSV files (*.csv)")

            if filename != None and filename != "":
                self.save_camera_calibration_to_csv(filename)
        except:
            self.create_error_dialog("Failed to save camera calibration as " + filename)

        self.update_enabled()
    
    def handle_save_as_clicked(self):
        print("saving settings")
        response = ArmCalibrationSRV()
        response = self.arm_calibration_client(self.arm_initial_guess[0], self.arm_initial_guess[1], self.arm_initial_guess[2])
        rospy.loginfo("FOUND: {0}".format(response.transform_matrix))

        self.arm_transform_matrix = response.transform_matrix
        self.arm_transform_matrix_step = response.transform_matrix_step

        try:
            filename, selected_filter = QFileDialog.getSaveFileName(self._widget, "Save Calibration", "/home", "CSV files (*.csv)")

            if filename != None and filename != "":
                self.save_calibration_to_csv(filename)
        except:
            self.create_error_dialog("Failed to save calibration as " + filename)
    
    def handle_start_RViz_clicked(self):
        print("starting RViz")
        response = ArmCalibrationSRV()
        response = self.arm_calibration_client(self.arm_initial_guess[0], self.arm_initial_guess[1], self.arm_initial_guess[2])
        rospy.loginfo("FOUND: {0}".format(response.transform_matrix))
        self.arm_transform_matrix = response.transform_matrix
        response = DisplayResultSRV()
        response = self.display_result_client(self.arm_transform_matrix, self.camera_transform_matrix)
        rospy.loginfo("FOUND: {0}".format(response.success))

        print(self.camera_matrix)
        print(self.camera_transform_matrix)
        print(self.arm_transform_matrix)

