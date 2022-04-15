#! usr/bin/env python

import sys
import json
import numpy as np

sys.path.append('../../src/')
from arm_calibration import ArmCalibration 

# read mock touch point transformation matrices into data object.
# mock data stored in './mock_transforms/tp*.json'
def get_mock_trans_matrx():
    data = {}

    for i in range(4):
        with open('./mock_transforms/tp' + str(i) + ".json") as mock:
            data[i] = json.load(mock)

    return data

# Call the calibrate_arm() function using the mock data
def test_calibrate_arm():
    calib = ArmCalibration()
    mock_data = get_mock_trans_matrx()
    result = calib.calibrate_arm(mock_data)

    print(result)


if __name__ == '__main__':
    test_calibrate_arm()
