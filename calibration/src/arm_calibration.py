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
        
        trans_guess = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, .53],
        [0, 0, 1, .035],
        [0, 0, 0, 1]])

        print(tp_data)

        return trans_guess


        
        




    '''
    def joint_angle_subscriber(self, angles):
        #This is where we will subscribe to the arm joint angles in the future
        pass

    def record_touchpoint_server(self, msg):
        #This is where the client in the GUI will tell us to record our joint angles and point (4 in total)
        pass

    def arm_transormation_martix_server(self, msg):
        #This is where the client in the GUI will query when it wants the transformation matrix from arm to table
        pass

    '''

if __name__ == "__main__":

    ArmCalibration().calibrate_arm(1)
