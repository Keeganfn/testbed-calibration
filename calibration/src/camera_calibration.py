#!/usr/bin/env python

import rospy
import sys

import numpy as np
from numpy import linalg as LA
import cv2 as cv
import glob
import tf
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from calibration.srv import CameraCalibrationSRV, CameraCalibrationSRVResponse


# Constants defined
boardX = 6 # how many squares are on the checkerboard in the x direction
boardY = 9 # how many squares are on the checkerboard in the y direction
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001) # termination criteria
arucoSideLength = 0.036 # 3.6 cm is the size of our arucos
arucoDictName = "DICT_5X5_1000" # dictionary name we're using, switch to user input at some point?
# map of possible aruco dictionaries
ARUCO_DICT = {
  "DICT_4X4_50": cv.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv.aruco.DICT_5X5_1000, # this is the one we using
  "DICT_6X6_50": cv.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv.aruco.DICT_ARUCO_ORIGINAL
}



class CameraCalibration:

    def __init__(self):
        #CvBridge can be used to convert an Image message into an opencv readable format
        self.bridge = CvBridge()
        self.rate = rospy.Rate(1)

        self.camera_photos = []
        self.aruco_photo = None

        #Ensures only 1 picture is taken
        self.take_picture_1 = False
        self.picture_error_1 = False

        #Subscribes to camera feed
        self.camera_sub_1 = rospy.Subscriber("camera_1/image_raw", Image, self.camera_1_callback, queue_size=1)
        #Service for take picture
        self.picture_service = rospy.Service("take_picture_srv", Trigger, self.take_picture_srv_callback)
        #service for camera calibration
        self.calibration_result_service = rospy.Service("camera_calibration_srv", CameraCalibrationSRV, self.camera_calibration_srv_callback)

    def take_picture_srv_callback(self, request):
        self.take_picture_1 = True
        #makes sure we take picture before returning
        while self.take_picture_1 == False:
            self.rate.sleep()
        #should return an error message if something went wrong, and a boolean signifying success or failure
        return self.picture_error_1, str(self.picture_error_1)

    #corresponds to camera1, multiple camera support will come later
    def camera_1_callback(self, msg):
        #msg is an Image message which you can save to a data structure
        if self.take_picture_1:
            #save message here
            rospy.loginfo("CAMERA CALIBRATION - PICTURE TAKEN ON CAMERA 1")
            # convert image to opencv image
            try:
                image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except CvBridgeError, e:
                rospy.logwarn('CAMERA CALIBRATION - CV Bridge error: {0}'.format(e))
            image = cv2.Canny(image, 100, 200)
            # now that we have opencv image, check if this should be added to camera calibration list (we haven't taken 30 photos yet)
            if len(self.camera_photos) < 30:
                self.camera_photos.append(image)
            # if we taken all of our calibration photos, write image to our special aruco photo (photo has checkerboard on table and arucos in frame, or height is provided)
            else:
                self.aruco_photo = image

            self.picture_error_1 = False
            self.take_picture_1 = False

    def camera_calibration_srv_callback(self, request):
        # initialize some empty lists and matrix sizes
        distortion = []
        camera_matrix = []
        transform_matrix = []
        camera_matrix_step = 3
        transform_matrix_step = 4
        height = 0
        #takes in height as an optional argument, will be -1 if none given
        rospy.loginfo("CAMERA CALIBRATION - HEIGHT IS: {0}".format(request.height))
        rospy.loginfo("CAMERA CALIBRATION - MATRICES GIVEN? {0}".format(request.existing))

        # get distortion and camera matrix from internal camera calibration
        distortion, camera_matrix = calibrate_internal(self, request)

        # time to calibrate table, first we need to find Z distance (height).
        height = calibrate_height(self, request, distortion, camera_matrix)

        # time for the ArUcos, used to calculate transformation matrix to the center of the table
        transform_matrix = calibrate_arucos(self, request, distortion, camera_matrix, height)

        # flatten any 2d matrices cause ros is dumb
        camera_matrix = camera_matrix.flatten()
        transform_matrix = transform_matrix.flatten()

        # print final values
        rospy.loginfo("CAMERA CALIBRATION - Final distortion: {0}".format(distortion))
        rospy.loginfo("CAMERA CALIBRATION - Final camera matrix: {0}".format(camera_matrix))
        rospy.loginfo("CAMERA CALIBRATION - Final transformation matrix: {0}".format(transform_matrix))

        return CameraCalibrationSRVResponse(distortion, camera_matrix_step, camera_matrix, transform_matrix_step, transform_matrix)



    def calibrate_internal(self, request):
        distortion = []
        camera_matrix = []
        # if we have not setup the camera and distortion matrix, calibrate them here
        if not request.existing:
            rospy.loginfo("CAMERA CALIBRATION - Calibrating camera")
            # let's start calibrating
            # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
            objp = np.zeros((boardX*boardY,3), np.float32)
            objp[:,:2] = np.mgrid[0:boardY,0:boardX].T.reshape(-1,2)
            # Arrays to store object points and image points from all the images.
            objpoints = [] # 3d point in real world space
            imgpoints = [] # 2d points in image plane.

            for img in self.camera_photos:
                rospy.loginfo("CAMERA CALIBRATION - Image: {0}".format(img))
                # get a grey scale version of the image
                gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
                # Find the chess board corners
                ret, corners = cv.findChessboardCorners(gray, (boardY,boardX), None)
                # If found, add object points, image points (after refining them)
                rospy.loginfo("CAMERA CALIBRATION - Image return val: {0}".format(ret))
                if ret == True:
                    rospy.loginfo("CAMERA CALIBRATION - Checkerboard corners found !")
                    objpoints.append(objp)
                    imgpoints.append(corners)
                    # only used if we want to uncomment the rest and show corners of board, not sure how/if this will work with how images are being read in
                    corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)

                    # Draw and display the corners, from the tutorial
                    #cv.drawChessboardCorners(frame, (boardY,boardX), corners2, ret)
                    #cv.imshow('img', frame)
                    #cv.waitKey(500)

            ret, camera_matrix, distortion, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        # otherwise, use the matrices given and unpack them
        elif request.existing:
            rospy.loginfo("CAMERA MAT: {0}".format(request.camera_matrix))
            rospy.loginfo("DISTORTION: {0}".format(request.distortion))
            camera_matrix = request.camera_matrix
            distortion = request.distortion
            camera_matrix = np.array([ [camera_matrix[1],camera_matrix[2],camera_matrix[3]], [camera_matrix[4],camera_matrix[5],camera_matrix[6]], [camera_matrix[7],camera_matrix[8],camera_matrix[9]] ])
            distortion = np.array([distortion[1],distortion[2],distortion[3],distortion[4],distortion[5]])

        return distortion, camera_matrix



    def calibrate_height(self, request, distortion, camera_matrix):
        height = -1
        # if we have not been given height (height == -1) use the photo to read the checkerboard to find height
        if request.height == -1:
            rospy.loginfo("CAMERA CALIBRATION - Calculating height with chessboard")
            if self.aruco_photo is None:
                rospy.loginfo("CAMERA CALIBRATION - No aruco photo found !!")
            else:
                gray = cv.cvtColor(self.aruco_photo, cv.COLOR_BGR2GRAY)
                ret, corners = cv.findChessboardCorners(gray, (boardY,boardX), None)

                if ret == True:
                    rospy.loginfo("CAMERA CALIBRATION - Checkerboard found on table")
                    objp = np.zeros((boardX*boardY,3), np.float32)
                    objp[:,:2] = np.mgrid[0:boardY,0:boardX].T.reshape(-1,2)
                    corners2 = cv.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

                    # Find the rotation and translation vectors.
                    _, rvec, tvec, inliers = cv.solvePnPRansac(objp, corners2, camera_matrix, distortion)

                    rospy.loginfo("CAMERA CALIBRATION - Found Translation Vector: {0}".format(tvec))
                    rospy.loginfo("CAMERA CALIBRATION - Found Rotation Vector: {0}".format(rvec))
                    rospy.loginfo("CAMERA CALIBRATION - New Height Value: {0}".format(tvec[2]))

                    height = tvec[2]
                else:
                    rospy.loginfo("CAMERA CALIBRATION - Couldn't find chessboard for height calculation !!")

        # otherwise, use height given to use
        else:
            rospy.loginfo("CAMERA CALIBRATION - Using given height")
            height = request.height
        return height



    def calibrate_arucos(self, request, distortion, camera_matrix, height):
        rospy.loginfo("CAMERA CALIBRATION - Final steps, pray to god this works")
        transform_matrix = []
        arucoDict = cv.aruco.Dictionary_get(ARUCO_DICT[arucoDictName])
        arucoParams = cv.aruco.DetectorParameters_create()

        if self.aruco_photo is None:
            rospy.loginfo("CAMERA CALIBRATION - No aruco photo found !!")
        else:
            # Detect ArUco markers in the video frame
            (corners, markerIDs, rejected) = cv.aruco.detectMarkers(self.aruco_photo, arucoDict, parameters=arucoParams,cameraMatrix=camera_matrix, distCoeff=distortion)

            # Check that at least one ArUco marker was detected
            if markerIDs is not None and len(markerIDs) == 4:
                # Draw a square around detected markers in the video frame, pretty sure this won't work anymore with how we're reading values
                # cv.aruco.drawDetectedMarkers(self.aruco_photo, corners, markerIDs)


                # Get the rotation and translation vectors
                rvecs, tvecs, objPoints = cv.aruco.estimatePoseSingleMarkers(
                    corners,
                    arucoSideLength,
                    camera_matrix,
                    distortion)

                for i, marker_id in enumerate(marker_ids):
                    rospy.loginfo("CAMERA CALIBRATION - marker_id: {0}  |  i: {1}".format(marker_id, i))
                    rospy.loginfo("CAMERA CALIBRATION - Rotation Vector: {0}".format(rvecs[i]))
                    rospy.loginfo("CAMERA CALIBRATION - Translation Vector: {0}".format(tvecs[i]))
                    # uncomment to draw axis on image, again not sure how this will work with how we're saving images
                    # cv.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)

                # find two opposite corners
                midpoint1_x = 0
                midpoint2_x = 0
                midpoint1_y = 0
                midpoint2_y = 0

                distance1 = LA.norm(tvecs[0] - tvecs[1])
                distance2 = LA.norm(tvecs[0] - tvecs[2])
                distance3 = LA.norm(tvecs[0] - tvecs[3])

                rospy.loginfo("CAMERA CALIBRATION - Distance 1: {0}".format(distance1))
                rospy.loginfo("CAMERA CALIBRATION - Distance 2: {0}".format(distance2))
                rospy.loginfo("CAMERA CALIBRATION - Distance 3: {0}".format(distance3))

                if distance1 >= distance2 and distance1 >= distance2:
                    midpoint1_x = (tvecs[0][0] + tvecs[1][0]) / 2
                    midpoint1_y = (tvecs[0][1] + tvecs[1][1]) / 2

                    midpoint2_x = (tvecs[2][0] + tvecs[3][0]) / 2
                    midpoint2_y = (tvecs[2][1] + tvecs[3][1]) / 2
                elif distance2 > distance3:
                    midpoint1_x = (tvecs[0][0] + tvecs[2][0]) / 2
                    midpoint1_y = (tvecs[0][1] + tvecs[2][1]) / 2

                    midpoint2_x = (tvecs[1][0] + tvecs[3][0]) / 2
                    midpoint2_y = (tvecs[1][1] + tvecs[3][1]) / 2
                else:
                    midpoint1_x = (tvecs[0][0] + tvecs[3][0]) / 2
                    midpoint1_y = (tvecs[0][1] + tvecs[3][1]) / 2

                    midpoint2_x = (tvecs[1][0] + tvecs[2][0]) / 2
                    midpoint2_y = (tvecs[1][1] + tvecs[2][1]) / 2


                # once we find two opposite corners of the table, calculate the midpoint
                center_rotation, _ = cv.Rodrigues(np.array([0,0,0]))
                center_translation = np.array([(midpoint1_x+midpoint2_x) / 2, (midpoint1_y+midpoint1_y) / 2, height])

                rospy.loginfo("CAMERA CALIBRATION - Camera-to-Table Rotation Matrix: {0}".format(center_rotation))
                rospy.loginfo("CAMERA CALIBRATION - Camera-to-Table Translation Vector: {0}".format(center_translation))

                # invert information before we combine into transform matrix
                center_rotation, center_translation = inversePerspective(center_rotation, center_translation)
                rospy.loginfo("CAMERA CALIBRATION - Table-to-Camera Rotation Matrix: {0}".format(center_rotation))
                rospy.loginfo("CAMERA CALIBRATION - Table-to-Camera Translation Vector: {0}".format(center_translation))

                # matrices must also be combined into 4x4 transformation matrix, also switches from rodrigues to quaternion
                transform_matrix = combine(center_rotation, center_translation)
                rospy.loginfo("CAMERA CALIBRATION - Table-to-Camera Transform Matrix: {0}".format(transform_matrix))

            else:
                rospy.loginfo("CAMERA CALIBRATION - Problem in identifying arucos !!")
                rospy.loginfo("CAMERA CALIBRATION - markerIDs: {0}".format(markerIDs))
        return transform_matrix


    def combine(rotation, tvec):
        combine = np.array([[0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 1]],
                            dtype=float)
        combine[:3, :3] = rotation

        # convert the matrix to a quaternion vector
        quaternion = tf.transformations.quaternion_from_matrix(combine)

        # convert quaternion vector to 3x3 matrix
        q0 = quaternion[0]
        q1 = quaternion[1]
        q2 = quaternion[2]
        q3 = quaternion[3]

        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)

        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)

        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1

        # 3x3 quaternion rotation matrix
        quat_mat = np.array([[r00, r01, r02],
                               [r10, r11, r12],
                               [r20, r21, r22]])

        # add to transform matrix
        combine[:3, :3] = quat_mat

        # add translation component
        combine[0][3] = tvec[0]
        combine[1][3] = tvec[1]
        combine[2][3] = tvec[2]

        return combine


    def inversePerspective(rotation, tvec):
        # transpose the matrix
        invRotation = np.matrix(rotation).T
        # inverses translation vector
        invTvec = np.dot(invRotation, np.matrix(-tvec))
        # return back to translation vector
        return invRotation, invTvec


if __name__ == "__main__":
    #initializes node and keeps spinning until roscore is shutdown
    rospy.init_node("camera_calibration", argv=sys.argv)
    calibrate = CameraCalibration()
    rospy.spin()
