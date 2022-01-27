import rospy


class ArmCalibration():

    def __init__(self):
        #Add anything you need in init and create as many functions as you need
        pass

    def joint_angle_subscriber(self, angles):
        #This is where we will subscribe to the arm joint angles in the future
        pass

    def record_touchpoint_server(self, msg):
        #This is where the client in the GUI will tell us to record our joint angles and point (4 in total)
        pass

    def arm_transormation_martix_server(self, msg):
        #This is where the client in the GUI will query when it wants the transformation matrix from arm to table
        pass

if __name__ == "__main__":
    ArmCalibration()
