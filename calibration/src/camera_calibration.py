import rospy


class CameraCalibration():

    def __init__(self):
        #Add anything you need in init and create as many functions as you need
        pass

    def camera_stream_subscriber(self, angles):
        #This is where we will subscribe to the arm joint angles in the future
        pass

    def camera_calibration_server(self, msg):
        #This is where we will hand off the camera calibration information, also is what will trigger calibration to begin
        pass

if __name__ == "__main__":
    CameraCalibration()
