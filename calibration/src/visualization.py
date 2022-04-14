#!/usr/bin/env python

import rospy
import sys

import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from calibration.srv import CameraCalibrationSRV, CameraCalibrationSRVResponse
from visualization_msgs.msg import Marker 



class VisualizeResult:

    BASE_FRAME = "world"
    TESTBED_FRAME = "testbed"
    CAMERA_FRAME = "camera_1"


    def __init__(self):
        self.rate = rospy.Rate(1)
        self.id = 0
        self.tf_pub = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=10)
        self.testbed_marker_pub = rospy.Publisher("testbed_marker", Marker, queue_size=10, latch=True)
        self.camera_marker_pub = rospy.Publisher("camera_marker", Marker, queue_size=10, latch=True)

        self.show_results()

    def pub_testbed_frame(self):
        testbed_frame = geometry_msgs.msg.TransformStamped()
        testbed_frame.header.frame_id = self.BASE_FRAME
        testbed_frame.header.stamp = rospy.Time.now()
        testbed_frame.child_frame_id = self.TESTBED_FRAME
        #replace with actual values
        testbed_frame.transform.translation.x = 0.0
        testbed_frame.transform.translation.y = -2.0
        testbed_frame.transform.translation.z = 0.0
        testbed_frame.transform.rotation.x = 0.0
        testbed_frame.transform.rotation.y = 0.0
        testbed_frame.transform.rotation.z = 0.0
        testbed_frame.transform.rotation.w = 1.0
        tfm = tf2_msgs.msg.TFMessage([testbed_frame])
        self.tf_pub.publish(tfm)

    def pub_camera_frame(self):
        camera_frame = geometry_msgs.msg.TransformStamped()
        camera_frame.header.frame_id = self.TESTBED_FRAME
        camera_frame.header.stamp = rospy.Time.now()
        camera_frame.child_frame_id = self.CAMERA_FRAME
        #replace with actual values
        camera_frame.transform.translation.x = 0.0
        camera_frame.transform.translation.y = 0.0
        camera_frame.transform.translation.z = 1.0
        camera_frame.transform.rotation.x = 0.0
        camera_frame.transform.rotation.y = 0.0
        camera_frame.transform.rotation.z = 0.0
        camera_frame.transform.rotation.w = 1.0
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
        table.scale.x = .5
        table.scale.y = .5
        table.scale.z = .01
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
