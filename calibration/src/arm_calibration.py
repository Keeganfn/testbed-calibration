#!/usr/bin/env python

import rospy
import sys
import numpy as np
import tf

from calibration.srv import ArmCalibrationSRV, ArmCalibrationSRVResponse
from calibration.srv import ArmRecordPointSRV, ArmRecordPointSRVResponse


class ArmCalibration:

    def __init__(self):
        #Service for record touchpoint
        self.touchpoint_service = rospy.Service("record_touchpoint_srv", ArmRecordPointSRV, self.record_touchpoint_srv_callback)
        #service for arm calibration
        self.calibration_result_service = rospy.Service("arm_calibration_srv", ArmCalibrationSRV, self.arm_calibration_srv_callback)

        # Dictionary to hold the transformation matrices from each touchpoint. Key: 0-4 (Touchpoint #), Value: Transform Matrix from enf effector - base.
        self.tp_data = dict()

    # *******************************************************************************************
    # @params: tp_data: dict object containing 4 Transformation matrices: One for each touchpoint.
    # Dictionary keys are 0-4, corresponding to the following touchpoint Transofmration matrices:
    # 0: Front left
    # 1: Back Left
    # 2: Back Right
    # 3: Front Right
    #
    # EXPECTS FORMAT: 
    # tp_data = {0: [<Trans_matrix>], 
    #            1: [<Trans_matrix>],
    #            2: [<Trans_matrix>],
    #            3: [<Trans_matrix>]}
    # 
    # @returns: a calibrated Transofmration matrix of the Robot arm's base in the world coordinate
    # frame.
    # *******************************************************************************************
    def calibrate_arm(self, tp_data):

        # returns a 2D array containing the translation portions of each touchpoint transform matrix.
        arr_translations = self.__getTranslations(tp_data)
        
        # Returns a calibrated transformation matrix from the robot arm base to the center of the testbed.
        newMatrix = self.__updateTransMatrix(arr_translations)
        
        print("Final Matrix:")
        print(newMatrix)

        # returns the transform matrix flattened to a 1D array
        return newMatrix.flatten()

    
    # Returns 4 arrays containing the XYZ coordinates of the end effector positions in the world frame.
    def __getTranslations(self, tp_data):
        
        arr_endEffInWorld = []
        
        # Loops through each touchpoint collected, adding the X-Y-Z values to an array.
        for i in range(0, len(tp_data)):
            endEffCoords = np.array([tp_data[i][0][-1], tp_data[i][1][-1], tp_data[i][2][-1], 1])
            arr_endEffInWorld.append(endEffCoords)

        return arr_endEffInWorld


    # Calibrates the XYZ location of the center of the testbed from the robot arm base.
    def __calibrateTranslations(self, arr_translations):

        x = 0
        y = 0
        z = 0
        
        # arr_len = 4, there are 4 touchpoints.
        arr_len = len(arr_translations)

        # Next, we average the X, Y, and Z components separately.
        for i in range(arr_len):
            x += arr_translations[i][0]
            y += arr_translations[i][1]
            z += arr_translations[i][2]

        # Store averaged XYZ components in these variables.
        dx = x / arr_len
        dy = y / arr_len
        dz = z / arr_len

        return (dx, dy, dz)

    # Calibrates the rotation portion of the transformation matrix.
    def __calibrateRotations(self, arr_translations):

        # Creates two vectors and subtracts them together.
        # These two vectors are the top-left - bottom-left rouchpoint,
        # and the top-rght - bottom-right tocuhpoint.
        # This gives us the x-vector.
        v_12 = arr_translations[1][:-1] - arr_translations[0][:-1]
        v_43 = arr_translations[2][:-1] - arr_translations[3][:-1]
        x_vec = (v_12 + v_43) / 2

        # Same thing, but subtracts the bottom-right - bottom-left,
        # and top-right - top-left. This gives us the y-vector.
        v_14 = arr_translations[3][:-1] - arr_translations[0][:-1]
        v_23 = arr_translations[2][:-1] - arr_translations[1][:-1]
        y_vec = (v_14 + v_23) / 2

        # Creates a y-prime vector, which is completely perpendicular to the X-vector.
        yp_vec = y_vec - np.dot(x_vec, y_vec) * x_vec

        # Performs cross-product between x and y vectors to obtain the z-vector.
        z_vec = np.cross(x_vec, yp_vec)

        # Creates 4x4 identity matrix.
        rotationMatrix = np.identity(4)

        # Creates the calibrated rotation matrix from the calculated vectors.
        rotationMatrix[0][:-1] = x_vec / np.linalg.norm(x_vec)
        rotationMatrix[1][:-1] = yp_vec / np.linalg.norm(yp_vec)
        rotationMatrix[2][:-1] = z_vec / np.linalg.norm(z_vec)

        print("Rotation Matrix: ")
        print(rotationMatrix)

        return rotationMatrix



    # Creates a calibrated transformation matrix from the robot arm base to the center of the table.
    def __updateTransMatrix(self, arr_translations):

        # Calculates the X, Y, and X distances from the robot arm base to the cetner of the table.
        # This is calculated by averaging the X, Y, and X components from each of the 4 translations
        # obtained from the 4 touchpoints.
        (dx, dy, dz) = self.__calibrateTranslations(arr_translations)

        # Creates a new rotations matrix based off the X-Y-Z coordinates of the 4 touchpoint values.
        # This creates a completely in-line rotation matrix from the robot arm base to the center of the table.
        newMatrix = self.__calibrateRotations(arr_translations)

        # Lastly, the calculated X, Y, and Z distances are added to the matrix.
        newMatrix[0][-1] = dx
        newMatrix[1][-1] = dy
        newMatrix[2][-1] = dz

        return newMatrix


    
    # Records the end effector location in the world frame. Stores transformation matrix in tp_data.
    def __recordTouchpoint(self, id):
        if rospy.has_param("calibration_config"): 
            #USE THESE VALUES HOWEVER YOU WANT
            config = rospy.get_param("calibration_config")
            base_frame = config["robot_base_link"]
            end_effector_frame = config["robot_end_effector"]
            print(end_effector_frame)
            print(base_frame)
        else:
            rospy.loginfo("CALIBRATION CONFIG NOT FOUND")
            base_frame = "j2s7s300_link_base"
            end_effector_frame = "j2s7s300_end_effector"

        listener = tf.TransformListener()

        # Need to loop infinitely until the transfrom listener decides to give us the transformation matrix.
        # uses tf to get the translation and rotation portions of the robot arm end effector to base.
        # We then create a transform matrix out of these, saving it the tp_data dictionary.
        while True:
            try:
                translation, rotation = listener.lookupTransform(base_frame, end_effector_frame, rospy.Time())
                transform_mat = listener.fromTranslationRotation(translation, rotation)

                # Stores the collected touchpoint transform matrix in the tp_data dictionary.
                self.tp_data[id] = transform_mat
                print("Touchpoint #" + str(id) + " recorded successfully:")
                print(transform_mat)

                return True
            # Keep looping until we get the transfrom matrix. Could take hundreds of iterations until success.
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    # The actual callback used via ROS to collect the touchpoint data.
    def record_touchpoint_srv_callback(self, request):
        #request contains the touchpoint location the arm is at
        rospy.loginfo("LOCATION IS: {0}".format(request.location))

        #call record touchpoint function, save result to data structure, update success and return
        success = self.__recordTouchpoint(request.location)

        print("TouchPoint Recorded Successfully? " + str(success))

        rospy.loginfo("RECORDED POINT")

        return ArmRecordPointSRVResponse(success)

    # The actual callback used by ROS to calculated the final transform matrix.
    def arm_calibration_srv_callback(self, request):
        # Collected the calibrated transfrom matrix.
        transform_matrix = self.calibrate_arm(self.tp_data)

        #should return these variables, step represents the dimension of the matrix (2d not supported), ie step=3 is a 3x3
        transform_matrix_step = 4

        # returned transformation matrix is falttened to 1D array. Can't send 2D array back.
        return ArmCalibrationSRVResponse(transform_matrix_step, transform_matrix)
    

if __name__ == "__main__":
    #initializes node and keeps spinning until roscore is shutdown
    rospy.init_node("arm_calibration", argv=sys.argv)
    calibrate = ArmCalibration()


    rospy.spin()
