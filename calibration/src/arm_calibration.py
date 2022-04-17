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
    def calibrate_arm(self, tp_data, initial_guess):
        
        x = initial_guess[0] or -.597
        y = initial_guess[1] or 0
        z = initial_guess[2] or .035

        # Estimated Transformation Matrix: Origin -> Robot arm Base
        estOriginToBase = np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]])

        print("initial guess matrix:")
        print(estOriginToBase)

        arr_translations = self.__getTranslations(estOriginToBase, tp_data)
        
        newMatrix = self.__updateTransMatrix(arr_translations, estOriginToBase)
        
        print("Final Matrix (After inverse):")
        print(newMatrix)

        # returns the transform matrix flattened to 1D array
        return np.array.flatten(newMatrix)

    # Returns a 1x4 vector containing the XYZ coordinates of the end effector in the world frame.
    def __getEndEffInWorld(self, originToBase, endEffToBase):

        test = np.matmul(originToBase, endEffToBase)
        endEffInWorld = np.matmul(test, np.transpose([0, 0, 0, 1]))

        return endEffInWorld
    
    # Returns 4 arrays containing the XYZ coordinates of the end effector positions in the world frame.
    def __getTranslations(self, originToBase, tp_data):
        
        arr_endEffInWorld = []

        for i in range(0, len(tp_data)):
            endEffCoords = self.__getEndEffInWorld(originToBase, tp_data[i])
            arr_endEffInWorld.append(endEffCoords)

        return arr_endEffInWorld


    # Calibrates the XYZ location of the robot arm base in the world frame.
    def __calibrateTranslations(self, arr_translations):

        x = 0
        y = 0
        z = 0
        
        #print(hello)
        arr_len = len(arr_translations)

        for i in range(arr_len):
            x += arr_translations[i][0]
            y += arr_translations[i][1]
            z += arr_translations[i][2]

        dx = x / arr_len
        dy = y / arr_len
        dz = z / arr_len

        return (dx, dy, dz)

    # Calibrates the rotation portion of the transformation matrix.
    def __calibrateRotations(self, arr_translations):

        print("arr_translations[0][:-1] = ")
        print(arr_translations[0][:-1])
        v_12 = arr_translations[1][:-1] - arr_translations[2][:-1]
        v_43 = arr_translations[0][:-1] - arr_translations[3][:-1]
        x_vec = (v_12 + v_43) / 2

        v_14 = arr_translations[0][:-1] - arr_translations[1][:-1]
        v_23 = arr_translations[3][:-1] - arr_translations[2][:-1]
        y_vec = (v_14 + v_23) / 2

        yp_vec = y_vec - np.dot(x_vec, y_vec) * x_vec


        z_vec = np.cross(x_vec, yp_vec)

        rotationMatrix = np.identity(4)

        rotationMatrix[0][:-1] = x_vec / np.linalg.norm(x_vec)
        rotationMatrix[1][:-1] = yp_vec / np.linalg.norm(yp_vec)
        rotationMatrix[2][:-1] = z_vec / np.linalg.norm(z_vec)

        print("Rotation Matrix: ")
        print(rotationMatrix)

        return rotationMatrix


    # Calibrates the original guess transformation matrix and returns the updated matrix.
    def __updateTransMatrix(self, arr_translations, oldMatrix):

        print("Before Calibrating Translation Portion: ")
        print(arr_translations)

        (dx, dy, dz) = self.__calibrateTranslations(arr_translations)

        newMatrix = self.__calibrateRotations(arr_translations)

        newMatrix[0][-1] = oldMatrix[0][-1] - dx
        newMatrix[1][-1] = oldMatrix[1][-1] - dy
        newMatrix[2][-1] = oldMatrix[2][-1] - dz

        print("Calibrated Matrix before Inverse:")
        print(newMatrix)

        inverse_matrix = np.linalg.inv(newMatrix)

        return newMatrix

        #print(dx, dy, dz)

    # Records the end effector location in the world frame. Stores transformation matrix in tp_data.
    def __recordTouchpoint(self, id):

        listener = tf.TransformListener()

        while True:
            try:
                translation, rotation = listener.lookupTransform('j2s7s300_link_base', 'j2s7s300_end_effector', rospy.Time())
                transform_mat = listener.fromTranslationRotation(translation, rotation)

                self.tp_data[id] = transform_mat
                print("Touchpoint #" + str(id) + "recorded successfully:")
                print(transform_mat)

                return True

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                
                return False


    def record_touchpoint_srv_callback(self, request):
        #request contains the touchpoint location the arm is at
        rospy.loginfo("LOCATION IS: {0}".format(request.location))

        #call record touchpoint function, save result to data structure, update success and return
        success = self.__recordTouchpoint(request.location)

        print("TouchPoint Recorded Successfully? " + str(success))

        rospy.loginfo("RECORDED POINT")

        return ArmRecordPointSRVResponse(success)


    def arm_calibration_srv_callback(self, request):
        rospy.loginfo("ARM ESTIMATED LOCATION: {0} {1} {2}".format(request.x, request.y, request.z))
        #request contains the estimated location given by the user
        initial_guess = [request.x, request.y, request.z]
        
        #DO CALCULATIONS HERE
        transform_matrix = self.calibrate_arm(self.tp_data, initial_guess)

        #should return these variables, step represents the dimension of the matrix (2d not supported), ie step=3 is a 3x3
        transform_matrix_step = 4

        #transform_matrix = [1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4]
        return ArmCalibrationSRVResponse(transform_matrix_step, transform_matrix)


if __name__ == "__main__":
    #initializes node and keeps spinning until roscore is shutdown
    rospy.init_node("arm_calibration", argv=sys.argv)
    calibrate = ArmCalibration()
    rospy.spin()
