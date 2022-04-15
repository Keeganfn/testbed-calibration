#!/usr/bin/env python

import rospy
import sys

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from calibration.srv import CameraCalibrationSRV, CameraCalibrationSRVResponse


class CameraCalibration:

    def __init__(self):
        #CvBridge can be used to convert an Image message into an opencv readable format
        self.bridge = CvBridge()
        self.rate = rospy.Rate(1)

        #Ensures only 1 picture is taken 
        self.take_picture_1 = False
        self.picture_error_1 = False

        #Subscribes to camera feed
        self.camera_sub_1 = rospy.Subscriber("camera_1/image_raw", Image, self.camera_1_callback, queue_size=1)
        #Service for take picture 
        self.picture_service = rospy.Service("take_picture_srv", Trigger, self.take_picture_srv_callback)
        #service for camera calibration
        self.calibration_result_service = rospy.Service("camera_calibration_srv", CameraCalibrationSRV, self.camera_calibration_srv_callback)

    #corresponds to camera1, multiple camera support will come later
    def camera_1_callback(self, msg):
        #msg is an Image message which you can save to a data structure
        if self.take_picture_1:
            #save message here
            rospy.loginfo("PICTURE TAKEN ON CAMERA 1")
            self.picture_error_1 = False
            self.take_picture_1 = False

    def take_picture_srv_callback(self, request):
        self.take_picture_1 = True 
        #makes sure we take picture before returning
        while self.take_picture_1 == False:
            self.rate.sleep()
        #should return an error message if something went wrong, and a boolean signifying success or failure
        return self.picture_error_1, str(self.picture_error_1)

    def camera_calibration_srv_callback(self, request):
        #takes in height as an optional argument, will be -1 if none given
        rospy.loginfo("EXISTING SETTINGS IS: {0}".format(request.existing_settings))
        rospy.loginfo("DISTORTION IS: {0}".format(request.distortion))
        rospy.loginfo("CAMERA MATRIX IS: {0}".format(request.camera_matrix))
        rospy.loginfo("CAMERA MATRIX STEP IS: {0}".format(request.camera_matrix_step))
        #should return these variables, step represents the dimension of the matrix (2d not supported), ie step=3 is a 3x3
        distortion = [1,2,3,4,5]
        camera_matrix = [1,2,3,1,2,3,1,2,3]
        camera_matrix_step = 3
        transform_matrix = [1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4]
        transform_matrix_step = 4
        return CameraCalibrationSRVResponse(distortion, camera_matrix_step, camera_matrix, transform_matrix_step, transform_matrix)

if __name__ == "__main__":
    #initializes node and keeps spinning until roscore is shutdown
    rospy.init_node("camera_calibration", argv=sys.argv)
    calibrate = CameraCalibration()
    rospy.spin()
