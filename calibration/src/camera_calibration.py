#!/usr/bin/env python

import rospy
import sys

import numpy as np
from numpy import linalg as LA
import cv2 as cv
import tf
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from calibration.srv import CameraCalibrationSRV, CameraCalibrationSRVResponse


# Constants defined
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001) # termination criteria

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

        # camera calibration config variables

        self.checkerboardSquareLength = 0.025
        self.arucoSideLength = 0.036 # 3.6 cm is the size of our arucos
        self.arucoDictName = "DICT_5X5_1000" # dictionary name we're using, switch to user input at some point?
        # map of possible aruco dictionaries

        self.topLeftID = 11
        self.topRightID = 12
        self.botLeftID = 10
        self.botRightID = 13

        self.boardX = 9 # how many squares are on the checkerboard in the x direction
        self.boardY = 6 # how many squares are on the checkerboard in the y direction

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
        return TriggerResponse(success=True, message="SUCCESS")
        #return self.picture_error_1, str(self.picture_error_1)

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
            # now that we have opencv image, check if this should be added to camera calibration list (we haven't taken 30 photos yet)
            if len(self.camera_photos) < 30:
                self.camera_photos.append(image)
           # if we taken all of our calibration photos, write image to our special aruco photo (photo has checkerboard on table and arucos in frame, or height is provided)
            else:
                self.aruco_photo = image
            print(self.camera_photos)

            self.picture_error_1 = False
            self.take_picture_1 = False

    def camera_calibration_srv_callback(self, request):
        # initialize some empty lists and matrix sizes
        if rospy.has_param("calibration_config"):
            # override camera config variables
            config = rospy.get_param("calibration_config")
            self.boardY = config["checkerboard_rows_default"]
            self.boardX = config["checkerboard_cols_default"]
            # self.checkerboardSquareLength = config["checkerboard_square_side_length"]
            self.arucoSideLength = config["aruco_sidelength"]
            self.arucoDictName = config["aruco_dict_used"]
            self.topLeftID = config["aruco_ids"][0]
            self.topRightID = config["aruco_ids"][1]
            self.botLeftID = config["aruco_ids"][2]
            self.botRightID = config["aruco_ids"][3]
        else:
            # using default values defined above
            rospy.loginfo("CALIBRATION CONFIG NOT FOUND, USING DEFAULT VALUES")

        distortion = []
        camera_matrix = []
        transform_matrix = []
        camera_matrix_step = 3
        transform_matrix_step = 4
        height = 0

        if request.existing_settings:
            print(self.camera_photos)
            self.aruco_photo = self.camera_photos[0]
            distortion = request.distortion
            camera_matrix = request.camera_matrix
            camera_matrix = np.array([ [camera_matrix[0],camera_matrix[1],camera_matrix[2]], [camera_matrix[3],camera_matrix[4],camera_matrix[5]], [camera_matrix[6],camera_matrix[7],camera_matrix[8]] ])
            distortion = np.array([distortion[0],distortion[1],distortion[2],distortion[3],distortion[4]])
        else:
            # get distortion and camera matrix from internal camera calibration
            distortion, camera_matrix = self.calibrate_internal()
        # time to calibrate table, first we need to find Z distance (height).
        height = self.calibrate_height(distortion, camera_matrix)
        # time for the ArUcos, used to calculate transformation matrix to the center of the table
        transform_matrix = self.calibrate_arucos(distortion, camera_matrix, height)
        # flatten any 2d matrices cause ros is dumb
        camera_matrix = camera_matrix.flatten()
        distortion = distortion.flatten()
        transform_matrix = transform_matrix.flatten()

        # print final values
        rospy.loginfo("CAMERA CALIBRATION - Final distortion: {0}".format(distortion))
        rospy.loginfo("CAMERA CALIBRATION - Final camera matrix: {0}".format(camera_matrix))
        rospy.loginfo("CAMERA CALIBRATION - Final transformation matrix: {0}".format(transform_matrix))
        return CameraCalibrationSRVResponse(distortion, camera_matrix_step, camera_matrix, transform_matrix_step, transform_matrix)



    def calibrate_internal(self):
        distortion = []
        camera_matrix = []
        # if we have not setup the camera and distortion matrix, calibrate them here
        rospy.loginfo("CAMERA CALIBRATION - Calibrating camera")
        # let's start calibrating
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((self.boardX*self.boardY,3), np.float32)
        objp[:,:2] = np.mgrid[0:self.boardY,0:self.boardX].T.reshape(-1,2)*self.checkerboardSquareLength
        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        for img in self.camera_photos:
            rospy.loginfo("CAMERA CALIBRATION - Image: {0}".format(img))
            # get a grey scale version of the image
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret, corners = cv.findChessboardCorners(gray, (self.boardY,self.boardX), None)
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

        rospy.loginfo("CAMERA MAT: {0}".format(camera_matrix))
        rospy.loginfo("DISTORTION: {0}".format(distortion))
        return distortion, camera_matrix



    def calibrate_height(self, distortion, camera_matrix):
        height = 0
        rospy.loginfo("CAMERA CALIBRATION - Calculating height with chessboard")
        if self.aruco_photo is None:
            rospy.loginfo("CAMERA CALIBRATION - No aruco photo found !!")
        else:
            gray = cv.cvtColor(self.aruco_photo, cv.COLOR_BGR2GRAY)
            ret, corners = cv.findChessboardCorners(gray, (self.boardY,self.boardX), None)

            if ret == True:
                rospy.loginfo("CAMERA CALIBRATION - Checkerboard found on table")
                objp = np.zeros((self.boardX*self.boardY,3), np.float32)
                objp[:,:2] = np.mgrid[0:self.boardY,0:self.boardX].T.reshape(-1,2)*self.checkerboardSquareLength
                corners2 = cv.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

                # Find the rotation and translation vectors.
                _, rvec, tvec, inliers = cv.solvePnPRansac(objp, corners2, camera_matrix, distortion)

                rospy.loginfo("CAMERA CALIBRATION - Found Translation Vector: {0}".format(tvec))
                rospy.loginfo("CAMERA CALIBRATION - Found Rotation Vector: {0}".format(rvec))
                rospy.loginfo("CAMERA CALIBRATION - New Height Value: {0}".format(tvec[2][0] * 0.0254)) # convert from inches to m

                height = tvec[2][0]
            else:
                rospy.loginfo("CAMERA CALIBRATION - Couldn't find chessboard for height calculation !!")
        return height



    def calibrate_arucos(self, distortion, camera_matrix, height):
        rospy.loginfo("CAMERA CALIBRATION - Final steps, pray to god this works")
        transform_matrix = []
        arucoDict = cv.aruco.Dictionary_get(ARUCO_DICT[self.arucoDictName])
        arucoParams = cv.aruco.DetectorParameters_create()

        if self.aruco_photo is None:
            rospy.loginfo("CAMERA CALIBRATION - No aruco photo found !!")
            transform_matrix = np.array([])
        else:
            # Detect ArUco markers in the video frame
            (corners, markerIDs, rejected) = cv.aruco.detectMarkers(self.aruco_photo, arucoDict, parameters=arucoParams)

            # Check that at least one ArUco marker was detected
            if markerIDs is not None and len(markerIDs) == 4:
                # Draw a square around detected markers in the video frame, pretty sure this won't work anymore with how we're reading values
                # cv.aruco.drawDetectedMarkers(self.aruco_photo, corners, markerIDs)


                # Get the rotation and translation vectors
                rvecs, tvecs = cv.aruco.estimatePoseSingleMarkers(
                    corners,
                    self.arucoSideLength,
                    camera_matrix,
                    distortion)

                # assume arucos are in set positions and set them
                topLeftT = []
                topRightT = []
                botLeftT = []
                botRightT = []

                for i, marker_id in enumerate(markerIDs):
                    # uncomment for debugging purposes
                    # rospy.loginfo("CAMERA CALIBRATION - marker_id: {0}  |  i: {1}".format(marker_id, i))
                    # rospy.loginfo("CAMERA CALIBRATION - Rotation Vector: {0}".format(rvecs[i]))
                    # rospy.loginfo("CAMERA CALIBRATION - Translation Vector: {0}".format(tvecs[i]))

                    if marker_id == self.topLeftID:
                        topLeftT = tvecs[i][0]
                    elif marker_id == self.topRightID:
                        topRightT = tvecs[i][0]
                    elif marker_id == self.botLeftID:
                        botLeftT = tvecs[i][0]
                    elif marker_id == self.botRightID:
                        botRightT = tvecs[i][0]

                    # uncomment to draw axis on image, again not sure how this will work with how we're saving images
                    # cv.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)

                # Averages the vectors between the top two points and the bottom two points, giving the x-vector
                v_tops = topRightT - topLeftT
                v_bots = botRightT - botLeftT
                x_vec = (v_tops + v_bots) / 2

                # Averages the vectors between the two points on each side, giving the y-vector
                v_lSide = topRightT - botRightT
                v_rSide = topLeftT - botLeftT
                y_vec = (v_lSide + v_rSide) / 2

                rospy.loginfo("CAMERA CALIBRATION - X Vector: {0}".format(x_vec))
                rospy.loginfo("CAMERA CALIBRATION - Y Vector: {0}".format(y_vec))

                # Force the y-vector to be orthogonal to the x-vector, creating the yprime-vector
                yp_vec = y_vec - np.dot(x_vec, y_vec) * x_vec

                rospy.loginfo("CAMERA CALIBRATION - Y Prime Vector: {0}".format(yp_vec))

                # Cross the x and yprime-vectors to get the z-vector that is orthogonal to the other two
                z_vec = np.cross(x_vec, yp_vec)

                rospy.loginfo("CAMERA CALIBRATION - Z Vector: {0}".format(z_vec))

                # Generate an identity matrix and replace the respective row with the x, yprime, and z-vectors after normalizing them
                center_rotation = np.identity(3)
                center_rotation[0] = x_vec / LA.norm(x_vec)
                center_rotation[1] = yp_vec / LA.norm(yp_vec)
                center_rotation[2] = z_vec / LA.norm(z_vec)

                # For translation, use given height, average x and y components of aruco markers
                x_tran = (topRightT[0] + topLeftT[0] + botRightT[0] + botLeftT[0]) / 4
                y_tran = (topRightT[1] + topLeftT[1] + botRightT[1] + botLeftT[1]) / 4
                center_translation = np.array([x_tran, y_tran, height])

                rospy.loginfo("CAMERA CALIBRATION - Camera-to-Table Rotation Matrix: {0}".format(center_rotation))
                rospy.loginfo("CAMERA CALIBRATION - Camera-to-Table Translation Vector: {0}".format(center_translation))

                # combine into 4x4 tranformation matrix (Camera-to-Table)
                transform_matrix = self.combine(center_rotation, center_translation)
                rospy.loginfo("CAMERA CALIBRATION - Table-to-Camera Transform Matrix: {0}".format(transform_matrix))

                # invert information before we combine into transform matrix
                transform_matrix = LA.inv(transform_matrix)
                rospy.loginfo("CAMERA CALIBRATION - Camera-to-Table Transform Matrix: {0}".format(transform_matrix))



            else:
                rospy.loginfo("CAMERA CALIBRATION - Problem in identifying arucos !!")
                rospy.loginfo("CAMERA CALIBRATION - markerIDs: {0}".format(markerIDs))
        return transform_matrix


    def combine(self, rotation, tvec):
        combine = np.array([[0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 1]],
                            dtype=float)
        combine[:3, :3] = rotation

        # add translation component
        combine[0][3] = tvec[0]
        combine[1][3] = tvec[1]
        combine[2][3] = tvec[2]
        return combine



if __name__ == "__main__":
    #initializes node and keeps spinning until roscore is shutdown
    rospy.init_node("camera_calibration", argv=sys.argv)
    calibrate = CameraCalibration()
    rospy.spin()
