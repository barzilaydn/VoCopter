#!/usr/bin/python
# encoding: utf-8

# TODO: Calibrate!


import sys
from functools import partial

from PySide import QtGui, QtCore, QtNetwork
from PySide.QtGui import *

from VoPilot_ui import Ui_MainWindow
from VoConsole import VoConsole, VoComms
from VoTest import VoTest
from VoControl import VoControl
import VoConfig
from VoCopter3D import VoCopter3D


class VoPilot(QMainWindow):
    newYaw = 0
    Yaw = 0
    Pitch = 0
    Roll = 0
    UpTime = 0
    State = VoConfig.SLEEP
    q = [0] * 4
    Heading = 0
    Temp = 0
    Altitude = 0
    Thrusts = [0] * 4
    BatLvl = 0

    def __init__(self, parent=None):
        super(VoPilot, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Setup the 3D viewer
        self.VoCopter3D_i = VoCopter3D("VoCopter.stl", self)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.VoCopter3D_i.sizePolicy().hasHeightForWidth())
        self.VoCopter3D_i.setSizePolicy(sizePolicy)
        self.VoCopter3D_i.setMinimumSize(QtCore.QSize(300, 300))
        self.VoCopter3D_i.setMaximumSize(QtCore.QSize(300, 300))
        self.ui.horizontalLayout_11.addWidget(self.VoCopter3D_i)
        self.VoCopter3D_i.ren.ResetCamera()

        # Setup the communications
        self.comms = VoComms(self)

        # Setup the extra windows
        self.VoConsole_i = VoConsole(self)
        self.VoTest_i = VoTest(self)
        # Bind to functions
        self.comms.signals.cmdAvailable.connect(self.newData)
        self.comms.signals.connectionChange.connect(self.connectionChange)
        self.comms.signals.logChange.connect(self.VoConsole_i.logChange)

        # Setup the controls
        self.controlAlive = True
        self.VoControl_i = VoControl(self)
        self.VoControl_i.signals.cmdAvailable.connect(self.sendControl)
        self.VoControl_i.signals.alert.connect(self.controlAlert)
        self.VoControl_i.start()

        self.ui.actionConsole.triggered.connect(self.openConsole)
        self.ui.connectBtn.setDisabled(False)
        # self.ui.connectBtn.released.connect(self.testModel)
        self.ui.connectBtn.released.connect(self.comms.toggleConnection)
        self.ui.vIP.setText('10.10.100.254')
        self.ui.vIP.textChanged.connect(self.IPvalid)
        self.ui.vIP.setPlaceholderText(VoConfig.DEFAULT_IP)

        self.updateUI()

        for s in xrange(VoConfig.NUM_OF_STATES):
            if s != 4:
                self.ui.verticalLayoutWidget_5.findChild(QPushButton, "state" + str(s)).setEnabled(False)
                self.ui.verticalLayoutWidget_5.findChild(QPushButton, "state" + str(s)).released.connect(
                    partial(self.changeState, s))

    def sendControl(self, thrust, yawChange, pitchChange, rollChange):
        self.newYaw = self.constrainAngle(yawChange + self.newYaw)
        newPitch = pitchChange
        newRoll = rollChange
        self.ui.Y.setText(str(self.newYaw))
        self.ui.P.setText(str(newPitch))
        self.ui.R.setText(str(newRoll))

        if self.State == VoConfig.MOVE:
            self.comms.writeCmd(VoConfig.MOVE, thrust, self.newYaw, newPitch, newRoll)

    # Create a "never ending" angle
    @staticmethod
    def constrainAngle(angle):
        if angle > 180:
            return angle - 360
        if angle < -180:
            return angle + 360
        else:
            return angle

    def controlAlert(self, data, isError):
        self.comms.writeToLog(data)
        if isError and self.comms.connected != 0:
            self.comms.ComDisconnect()  # Disconnect if no joystick available

    def IPvalid(self):
        # Use QHostAddress to check if address is valid:
        self.ui.connectBtn.setEnabled(
            QtNetwork.QHostAddress().setAddress(self.ui.vIP.text())
        )

    def changeState(self, s):
        if s == VoConfig.TEST:
            self.VoTest_i.show()
            self.VoTest_i.syncTest()
        elif s == VoConfig.CALIBRATE:
            pass
        else:
            self.comms.writeCmd(s)

    # STATUS FORMAT: ' UP_TIME ; STATE ; 100*q[0],100*q[1],100*q[2],100*q[3],HEADING,ALTITUDE ; FRONT_LEFT,FRONT_RIGHT,BACK_LEFT,BACK_RIGHT ; BAT_LVL '
    def newData(self, cmd, params):
        # Confirm receiving
        if cmd != VoConfig.Q_ACK:
            self.comms.writeCmd(VoConfig.Q_ACK, 0)

        # if command is a STATUS, update local data
        if cmd == VoConfig.Q_STATUS:
            self.UpTime = params[0]
            self.State = params[1]
            # self.q =
            self.Yaw = params[2]
            self.Pitch = params[3]
            self.Roll = params[4]
            self.Temp = params[5]
            self.Heading = params[6]
            self.Altitude = params[7]
            self.Thrusts = params[8:12]
            self.BatLvl = params[12]

            self.updateUI()

        # if command is a ALERT, update the status bar.
        elif cmd == VoConfig.Q_ALERT:
            if params[0] == VoConfig.Q_LOW_BAT:
                self.comms.writeToLog('ALERT: Low Battery!')
            if params[0] == VoConfig.Q_TIME_OUT:
                self.comms.writeToLog('ALERT: You were away for too long, VoCopter went to sleep!')
            else:
                self.comms.writeToLog('ALERT: Unknown Error #{}'.format(params[0]))

        # if command is a SLEEPING alert, update local data.
        elif cmd == VoConfig.Q_SLEEPING:
            self.State = VoConfig.SLEEP
            self.updateUI()

        elif cmd == VoConfig.Q_TEST:
            self.State = VoConfig.TEST
            self.updateUI()
            if params[0] == VoConfig.Q_TEST_IMU:
                self.VoTest_i.RawValues(params)
            elif params[0] == VoConfig.Q_TEST_PID:
                self.VoTest_i.PID(params)
            else:
                self.comms.writeToLog("Unkown test received: {}.".format(cmd))

        elif cmd == VoConfig.Q_ACK:
            self.comms.goodConnection = True
            self.comms.dataReceived = True
            self.comms.writeToLog("VoCopter acknowledged command.")

        else:
            self.comms.writeToLog("ERROR: Unknown command was received. Command ID: {}".format(cmd))

    def updateUI(self):
        self.ui.batLvl.setProperty("value", self.BatLvl)
        self.ui.m1.setProperty("value", self.Thrusts[0] / 2.55)
        self.ui.m2.setProperty("value", self.Thrusts[1] / 2.55)
        self.ui.m3.setProperty("value", self.Thrusts[2] / 2.55)
        self.ui.m4.setProperty("value", self.Thrusts[3] / 2.55)
        self.ui.connectionTime.setText(str(self.UpTime))
        self.ui.alt.setText(str(self.Altitude))
        # self.fixedQ = [self.q[i] + VoConfig.QOffset[i] for i in xrange(len(self.q))]
        # self.VoCopter3D_i.rotateToQ(self.fixedQ)
        self.VoCopter3D_i.rotateToYPR(self.Yaw, self.Pitch, self.Roll)

        self.ui.head.setText(str(self.Heading))
        self.ui.yaw.setText(str(self.Yaw))
        self.ui.pitch.setText(str(self.Pitch))
        self.ui.roll.setText(str(self.Roll))

        # Enable / Disable state buttons:
        for s in xrange(VoConfig.NUM_OF_STATES):
            if s == self.State:
                self.ui.verticalLayoutWidget_5.findChild(QPushButton, "state" + str(s)).setEnabled(False)
            elif s != 4 and s != 6:  # No button for state 4, state 6 not implemented yet
                self.ui.verticalLayoutWidget_5.findChild(QPushButton, "state" + str(s)).setEnabled(True)

    def connectionChange(self, connected):
        self.VoConsole_i.ui.sendCmd.setEnabled(False)
        self.ui.connectBtn.setEnabled(False)
        self.ui.vIP.setDisabled(connected)

        if connected == 1:
            self.changeState(VoConfig.FLY)
            self.VoConsole_i.ui.cmd.setEnabled(True)
            self.ui.connectBtn.setText("Disconnect")
            self.ui.state0.setEnabled(True)
            self.ui.state2.setEnabled(True)
            self.ui.state3.setEnabled(True)
            self.ui.state5.setEnabled(True)
            self.ui.state6.setEnabled(True)
        elif connected == 2:
            self.ui.connectBtn.setText("Connecting...")
        elif connected == 3:
            self.ui.connectBtn.setText("Disconnecting...")
        else:
            self.ui.connectBtn.setText("Connect")
            self.ui.connectBtn.setEnabled(True)
            self.VoConsole_i.ui.sendCmd.setEnabled(False)
            self.VoConsole_i.ui.cmd.setEnabled(False)
            self.ui.state0.setEnabled(False)
            self.ui.state2.setEnabled(False)
            self.ui.state3.setEnabled(False)
            self.ui.state5.setEnabled(False)
            self.ui.state6.setEnabled(False)

    def openConsole(self):
        self.VoConsole_i.show()

    def testModel(self):
        line = str(self.ui.vIP.text()).split(';')
        self.Yaw = float(line[0])
        self.Pitch = float(line[1])
        self.Roll = float(line[2])
        self.updateUI()

if __name__ == "__main__":
    QtGui.QApplication.setStyle("plastique")
    app = QtGui.QApplication(sys.argv)
    VoPilot_i = VoPilot()

    # Handle close event to correctly terminate the control thread
    def exitHandler():
        print 'Bye!'
        VoPilot_i.controlAlive = False
        if VoPilot_i.State != VoConfig.SLEEP:
            VoPilot_i.comms.writeCmd(VoConfig.Q_ACK, 0)
        VoPilot_i.VoControl_i.wait()

    app.aboutToQuit.connect(exitHandler)
    VoPilot_i.show()
    sys.exit(app.exec_())
