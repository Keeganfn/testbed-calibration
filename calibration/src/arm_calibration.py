#! usr/bin/env python

import rospy
import numpy as np

class ArmCalibration:

    def __init__(self):
        #Add anything you need in init and create as many functions as you need
        pass

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
        
        # Estimated Transformation Matrix: Origin -> Robot arm Base
        estOriginToBase = np.array([
        [1, 0, 0, -.597],
        [0, 1, 0, 0],
        [0, 0, 1, .035],
        [0, 0, 0, 1]])

        arr_translations = self.__getTranslations(estOriginToBase, tp_data)
        
        newMatrix = self.__updateTransMatrix(arr_translations, estOriginToBase)
        
        return newMatrix

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


        return rotationMatrix


    # Calibrates the original guess transformation matrix and returns the updated matrix.
    def __updateTransMatrix(self, arr_translations, oldMatrix):

        (dx, dy, dz) = self.__calibrateTranslations(arr_translations)

        newMatrix = self.__calibrateRotations(arr_translations)

        newMatrix[0][-1] = oldMatrix[0][-1] - dx
        newMatrix[1][-1] = oldMatrix[1][-1] - dy
        newMatrix[2][-1] = oldMatrix[2][-1] - dz

        return newMatrix

        #print(dx, dy, dz)


if __name__ == "__main__":

    print("Run me via the test script, arm_calibration_test.py")
