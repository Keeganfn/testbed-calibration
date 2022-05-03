#!/usr/bin/env python

import rospy
import sys
import tf
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import numpy as np

from calibration.srv import DisplayResultSRV, DisplayResultSRVResponse
from visualization_msgs.msg import Marker 



class VisualizeResult:

    BASE_FRAME = "world"
    TESTBED_FRAME = "testbed"
    CAMERA_FRAME = "camera_1"
    TESTBED_L = .5
    TESTBED_W = .5
    TESTBED_H = .01


    def __init__(self):
        self.testbed_translation = None
        self.testbed_rotation = None
        self.arm_translation = None
        self.arm_rotation = None

        self.tf_pub = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=10)
        self.display_service = rospy.Service("display_result_srv", DisplayResultSRV, self.display_result_srv_callback)
        self.testbed_marker_pub = rospy.Publisher("testbed_marker", Marker, queue_size=10, latch=True)
        self.camera_marker_pub = rospy.Publisher("camera_marker", Marker, queue_size=10, latch=True)
        
    def display_result_srv_callback(self, request):
        if rospy.has_param("calibration_config"): 
            #USE THESE VALUES HOWEVER YOU WANT
            config = rospy.get_param("calibration_config")
            self.BASE_FRAME = config["robot_base_link"]
            self.TESTBED_L = config["testbed_size"][0]
            self.TESTBED_W = config["testbed_size"][1]
        else:
            rospy.loginfo("CALIBRATION CONFIG NOT FOUND")

        rospy.loginfo("ARM TO TESTBED IS: {0}".format(request.arm_to_testbed))
        rospy.loginfo("TESTBED TO CAMERA IS: {0}".format(request.testbed_to_camera))
        arm_to_testbed = request.arm_to_testbed
        testbed_to_camera = request.testbed_to_camera

        arm_to_testbed_transform = np.array(arm_to_testbed).reshape(4,4)
        testbed_to_camera_transform = np.array(testbed_to_camera).reshape(4,4)
        self.testbed_translation = tf.transformations.translation_from_matrix(arm_to_testbed_transform)
        self.testbed_rotation = tf.transformations.quaternion_from_matrix(arm_to_testbed_transform)
        self.camera_translation = tf.transformations.translation_from_matrix(testbed_to_camera_transform)
        self.camera_rotation = tf.transformations.quaternion_from_matrix(testbed_to_camera_transform)
        self.show_results()
        success = True
        return DisplayResultSRVResponse(success)


    def pub_testbed_frame(self):
        testbed_frame = geometry_msgs.msg.TransformStamped()
        testbed_frame.header.frame_id = self.BASE_FRAME
        testbed_frame.header.stamp = rospy.Time.now()
        testbed_frame.child_frame_id = self.TESTBED_FRAME
        #replace with actual values
        testbed_frame.transform.translation.x = self.testbed_translation[0]
        testbed_frame.transform.translation.y = self.testbed_translation[1]
        testbed_frame.transform.translation.z = self.testbed_translation[2]
        testbed_frame.transform.rotation.x = self.testbed_rotation[0]
        testbed_frame.transform.rotation.y = self.testbed_rotation[1]
        testbed_frame.transform.rotation.z = self.testbed_rotation[2]
        testbed_frame.transform.rotation.w = self.testbed_rotation[3]
        tfm = tf2_msgs.msg.TFMessage([testbed_frame])
        self.tf_pub.publish(tfm)

    def pub_camera_frame(self):
        camera_frame = geometry_msgs.msg.TransformStamped()
        camera_frame.header.frame_id = self.TESTBED_FRAME
        camera_frame.header.stamp = rospy.Time.now()
        camera_frame.child_frame_id = self.CAMERA_FRAME
        #replace with actual values
        camera_frame.transform.translation.x = self.camera_translation[0]
        camera_frame.transform.translation.y = self.camera_translation[1]
        camera_frame.transform.translation.z = self.camera_translation[2]
        camera_frame.transform.rotation.x = self.camera_rotation[0]
        camera_frame.transform.rotation.y = self.camera_rotation[1]
        camera_frame.transform.rotation.z = self.camera_rotation[2]
        camera_frame.transform.rotation.w = self.camera_rotation[3]
        tfm = tf2_msgs.msg.TFMessage([camera_frame])
        self.tf_pub.publish(tfm)


    def show_results(self):
        self.pub_testbed_marker()
        self.pub_camera_marker()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            self.pub_testbed_frame()
            self.pub_camera_frame()

    def pub_testbed_marker(self):
        table = Marker()
        table.header.frame_id = self.TESTBED_FRAME
        table.type = table.CUBE
        table.id = 0
        table.action = table.ADD
        table.scale.x = self.TESTBED_L
        table.scale.y = self.TESTBED_W
        table.scale.z = self.TESTBED_H
        table.color.r = 0
        table.color.g = 1
        table.color.b = 0
        table.color.a = 1
        table.pose.position.x = 0
        table.pose.position.y = 0
        table.pose.position.z = 0
        self.testbed_marker_pub.publish(table)

    def pub_camera_marker(self):
        camera = Marker()
        camera.header.frame_id = self.CAMERA_FRAME
        camera.type = camera.CUBE
        camera.id = 1
        camera.action = camera.ADD
        camera.scale.x = .2
        camera.scale.y = .2
        camera.scale.z = .2
        camera.color.r = 0
        camera.color.g = 0
        camera.color.b = 1
        camera.color.a = 1
        camera.pose.position.x = 0
        camera.pose.position.y = 0
        camera.pose.position.z = 0
        self.camera_marker_pub.publish(camera)



if __name__ == "__main__":
    #initializes node and keeps spinning until roscore is shutdown
    rospy.init_node("visualize_result", argv=sys.argv)
    calibrate = VisualizeResult()
    rospy.spin()
