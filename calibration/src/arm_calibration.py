#!/usr/bin/env python

import rospy
import sys
import numpy as np

from calibration.srv import ArmCalibrationSRV, ArmCalibrationSRVResponse
from calibration.srv import ArmRecordPointSRV, ArmRecordPointSRVResponse


class ArmCalibration:

    def __init__(self):
        #Service for record touchpoint
        self.touchpoint_service = rospy.Service("record_touchpoint_srv", ArmRecordPointSRV, self.record_touchpoint_srv_callback)
        #service for arm calibration
        self.calibration_result_service = rospy.Service("arm_calibration_srv", ArmCalibrationSRV, self.arm_calibration_srv_callback)


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

    def record_touchpoint_srv_callback(self, request):
        #request contains the touchpoint location the arm is at
        rospy.loginfo("LOCATION IS: {0}".format(request.location))
        #call record touchpoint function, save result to data structure, update success and return
        success = True
        rospy.loginfo("RECORDED POINT")
        return ArmRecordPointSRVResponse(success)

    def arm_calibration_srv_callback(self, request):
        rospy.loginfo("ARM ESTIMATED LOCATION: {0} {1} {2}".format(request.x, request.y, request.z))
        #request contains the estimated location given by the user
        initial_guess = [request.x, request.y, request.z]
        
        #DO CALCULATIONS HERE

        #should return these variables, step represents the dimension of the matrix (2d not supported), ie step=3 is a 3x3
        transform_matrix_step = 4
        transform_matrix = [1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4]
        return ArmCalibrationSRVResponse(transform_matrix_step, transform_matrix)

    

if __name__ == "__main__":
    #initializes node and keeps spinning until roscore is shutdown
    rospy.init_node("arm_calibration", argv=sys.argv)
    calibrate = ArmCalibration()
    rospy.spin()
