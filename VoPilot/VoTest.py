#!/usr/bin/python
# encoding: utf-8

from PySide import QtGui, QtCore, QtNetwork
from PySide.QtCore import *
from PySide.QtGui import *
from functools import partial
from VoTest_ui import Ui_VoTest
import VoConfig

class VoTest(QMainWindow):
    def __init__(self, parent=None):
        super(VoTest, self).__init__(parent)
        
        self.parent = parent
        
        self.ui = Ui_VoTest()
        self.ui.setupUi(self)
        
        self.test = VoConfig.Q_TEST_IMU
        self.syncTest()
        
        self.ui.RawValues.setChecked(True)
        self.ui.RawValues.clicked.connect(partial(self.testChange, VoConfig.Q_TEST_IMU))
        self.ui.PID.setChecked(False)
        self.ui.PID.clicked.connect(partial(self.testChange, VoConfig.Q_TEST_PID))
        self.ui.MotorsTest.setChecked(False)
        self.ui.MotorsTest.clicked.connect(partial(self.testChange, VoConfig.Q_TEST_MOTORS))
        
        self.ui.baseThrust.valueChanged.connect(self.syncTest)
        self.ui.m1.valueChanged.connect(self.syncTest)
        self.ui.m2.valueChanged.connect(self.syncTest)
        self.ui.m3.valueChanged.connect(self.syncTest)
        self.ui.m4.valueChanged.connect(self.syncTest)
        self.ui.m.valueChanged.connect(self.setBaseThrust)
                
    def testChange(self, id):
        # Don't allow for no test to be checked
        self.test = id
        if id == VoConfig.Q_TEST_IMU:
            self.ui.RawValues.setChecked(True)
            self.ui.PID.setChecked(False)
            self.ui.MotorsTest.setChecked(False)
        if id == VoConfig.Q_TEST_PID:
            self.ui.RawValues.setChecked(False)
            self.ui.PID.setChecked(True)
            self.ui.MotorsTest.setChecked(False)
        if id == VoConfig.Q_TEST_MOTORS:
            self.ui.RawValues.setChecked(False)
            self.ui.PID.setChecked(False)
            self.ui.MotorsTest.setChecked(True)
    
        self.syncTest()
    
    def setBaseThrust(self):
        base = self.ui.m.value()
        self.ui.mVal.setText(str(base))
        self.ui.m1.setValue(base)
        self.ui.m2.setValue(base)
        self.ui.m3.setValue(base)
        self.ui.m4.setValue(base)
    
    def syncTest(self):
        if self.test == VoConfig.Q_TEST_IMU: # IMU
            self.parent.comms.writeCmd(VoConfig.TEST, VoConfig.Q_TEST_IMU, 0)
        
        elif self.test == VoConfig.Q_TEST_PID: # PID
            self.parent.comms.writeCmd(VoConfig.TEST, VoConfig.Q_TEST_PID, self.ui.baseThrust.value())
        
        elif self.test == VoConfig.Q_TEST_MOTORS: # Motors_individual
            self.parent.comms.writeCmd(VoConfig.TEST, VoConfig.Q_TEST_MOTORS,
                                                                    self.ui.m1.value(),
                                                                    self.ui.m2.value(),
                                                                    self.ui.m3.value(),
                                                                    self.ui.m4.value())                                                                    
        self.updateValues()
    
    def updateValues(self):
        self.ui.baseThrustVal.setText(str(self.ui.baseThrust.value()))
        self.ui.m1Val.setText(str(self.ui.m1.value()))
        self.ui.m2Val.setText(str(self.ui.m2.value()))
        self.ui.m3Val.setText(str(self.ui.m3.value()))
        self.ui.m4Val.setText(str(self.ui.m4.value()))
        
    def RawValues(self, params):
        try:
            self.ui.accX.setText(str(params[1]))
            self.ui.accY.setText(str(params[2]))
            self.ui.accZ.setText(str(params[3]))
            self.ui.gyroX.setText(str(params[4]))
            self.ui.gyroY.setText(str(params[5]))
            self.ui.gyroZ.setText(str(params[6]))
            self.ui.magX.setText(str(params[7]))
            self.ui.magY.setText(str(params[8]))
            self.ui.magZ.setText(str(params[9]))
            self.ui.temp.setText(str(params[10]))
            self.ui.motionDetect.setText(str(params[11]))
            self.ui.libVer.setText(str(params[12]))
        except:
            self.parent.comms.writeToLog('ERROR: Not valid params for Raw Values test.')          
    
    def PID(self, params):
        try:
            self.ui.yawOut.setText(str(params[1]))
            self.ui.pitchOut.setText(str(params[2]))
            self.ui.rollOut.setText(str(params[3]))
        except:
            self.parent.comms.writeToLog('ERROR: Not valid params for PID test.')   