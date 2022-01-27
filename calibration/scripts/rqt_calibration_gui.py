#!/usr/bin/env python

import sys

from calibration.rqt_calibration_gui.calibrationGUI import CalibrationGUI
from rqt_gui.main import Main

plugin = 'rqt_calibration_gui'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin
