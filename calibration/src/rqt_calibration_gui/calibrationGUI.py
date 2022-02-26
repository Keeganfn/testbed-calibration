import os
import rospy
import actionlib
import rospkg

from argparse import ArgumentParser
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtGui import QBrush, QColor, QIcon
from python_qt_binding.QtWidgets import QWidget, QAbstractItemView, QTableWidgetItem, QAbstractScrollArea, QMessageBox, QApplication, QDialog

from std_srvs.srv import Trigger, TriggerResponse
from calibration.srv import CameraCalibrationSRV, CameraCalibrationResponse, CameraCalibrationSRVRequest
from calibration.srv import ArmCalibrationSRV, ArmCalibrationSRVResponse, ArmCalibrationSRVRequest
from calibration.srv import ArmRecordPointSRV, ArmRecordPointSRVResponse, ArmRecordPointSRVRequest



class ImportDialog(QDialog):
    def __init__(self):
        super(ImportDialog, self).__init__()
        self.setObjectName('ImportDialog')
        ui_file = os.path.join(rospkg.RosPack().get_path('calibration'), 'resource', 'import.ui')
        loadUi(ui_file, self)

        self.buttonBox.accepted.connect(self.handle_ok_clicked)
        self.buttonBox.rejected.connect(self.handle_cancel_clicked)

        self.importTBComboBox.currentIndexChanged.connect(self.handle_import_TB_change)
        self.importCamComboBox.currentIndexChanged.connect(self.handle_import_cam_change)

    def handle_import_TB_change(self, i):
        print("Testbed import settings is:", i, "Value:", self.importTBComboBox.currentText())

    def handle_import_cam_change(self, i):
        print("Cam import settings is:", i, "Value:", self.importCamComboBox.currentText())
    
    def handle_ok_clicked(self):
        print("Ok clicked")

    def handle_cancel_clicked(self):
        print("Cancel clicked")

class SaveDialog(QDialog):
    def __init__(self):
        super(SaveDialog, self).__init__()
        self.setObjectName('SaveDialog')
        ui_file = os.path.join(rospkg.RosPack().get_path('calibration'), 'resource', 'save.ui')
        loadUi(ui_file, self)

        self.buttonBox.accepted.connect(self.handle_save_clicked)
        self.buttonBox.rejected.connect(self.handle_cancel_clicked)

    def handle_save_clicked(self):
        print("Save clicked")
        print("Filename: " + self.filenameLineEdit.text())

    def handle_cancel_clicked(self):
        print("Cancel clicked")

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


        self.total_pictures = 0
        self.z_dist = 0
        self.testbed_model = None
        self.arm_model = None
        self.top_camera = None
        self.side_camera = None

        #self._widget.pictureButton.setEnabled(False)
        #self._widget.cameraPoseButton.setEnabled(False)
        #self._widget.zDistSpinBox.setEnabled(False)
        #self._widget.saveCamSettingsButton.setEnabled(False)
        #self._widget.xSpinBox.setEnabled(False)
        #self._widget.ySpinBox.setEnabled(False)
        #self._widget.zSpinBox.setEnabled(False)
        #self._widget.upperLeftButton.setEnabled(False)
        #self._widget.upperRightButton.setEnabled(False)
        #self._widget.lowerLeftButton.setEnabled(False)
        #self._widget.lowerRightButton.setEnabled(False)
        #self._widget.saveAsButton.setEnabled(False)
        #self._widget.startRVizButton.setEnabled(False)


        # Get zdist value manually 
        #self._widget.zDistSpinBox.value()

        #Inital guess values
        #self._widget.xSpinBox.value()
        #self._widget.ySpinBox.value()
        #self._widget.zSpinBox.value()
        

    
    def handle_import_clicked(self):
        print("importing internal camera calibration settings")
        import_dialog = ImportDialog()
        import_dialog.exec_()

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
                self._widget.pictureButton.setEnabled(False)

    def handle_calibrate_camera_pose_clicked(self):
        # Set label content
        self._widget.zDistFoundLabel.setText("Z Distance Found: "+ str(self.z_dist))
        print("calibrating camera pose")
        response = CameraCalibrationResponse()
        response = self.camera_calibration_client(self.z_dist)
        rospy.loginfo("FOUND {0} {1} {2}".format(response.distortion, response.camera_matrix, response.transform_matrix))

    def record_touchpoint(self,location):
        response = ArmRecordPointSRVResponse()
        response = self.record_touchpoint_client(location)
        rospy.loginfo("TOUCHPOINT RECORD: {0}".format(response.success))

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
        self.record_touchpoint(1)
        self._widget.upperLeftLabel.setText("Status: Done")
        print("getting upper left touchpt")

    def handle_upper_right_touchpt_clicked(self):
        self.record_touchpoint(2)
        self._widget.upperRightLabel.setText("Status: Done")
        print("getting upper right touchpt")

    def handle_lower_left_touchpt_clicked(self):
        self.record_touchpoint(0)
        self._widget.lowerLeftLabel.setText("Status: Done")
        print("getting lower left touchpt")

    def handle_lower_right_touchpt_clicked(self):
        self.record_touchpoint(3)
        self._widget.lowerRightLabel.setText("Status: Done")
        print("getting lower right touchpt")
    
    def handle_save_camera_settings_clicked(self):
        print("saving camera settings")
    
    def handle_view_top_clicked(self):
        print("viewing top camera")
    
    def handle_view_side_clicked(self):
        print("viewing side camera")
    
    def handle_save_as_clicked(self):
        print("saving settings")
        response = ArmCalibrationSRV()
        response = self.arm_calibration_client(1,2,3)
        rospy.loginfo("FOUND: {0}".format(response.transform_matrix))
        save_dialog = SaveDialog()
        save_dialog.exec_()
    
    def handle_start_RViz_clicked(self):
        print("starting RViz")
