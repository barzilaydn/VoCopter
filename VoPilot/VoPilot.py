#!/usr/bin/python
# encoding: utf-8

# TODO:
# Calibrate

import sys, numpy, math
from PySide import QtGui, QtCore, QtNetwork
from PySide.QtGui import *
from functools import partial
from VoPilot_ui import Ui_MainWindow
from VoConsole import VoConsole, VoComms
from VoTest import VoTest
from VoControl import VoControl
import VoConfig
from VoCopter3D import VoCopter3D

class VoPilot(QMainWindow):
    Yaw = 0
    Pitch = 0
    Roll = 0
    UpTime = 0
    State = VoConfig.SLEEP
    q = [0]*4
    Heading = 0
    Altitude = 0
    Thrusts = [0]*4
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
        self.ui.connectBtn.released.connect(self.comms.toggleConnection)
        self.ui.vIP.textChanged.connect(self.IPvalid)
        self.ui.vIP.setPlaceholderText(VoConfig.DEFAULT_IP)
        
        self.updateUI()
        
        for s in xrange(VoConfig.NUM_OF_STATES):
            if s != 4:
                self.ui.verticalLayoutWidget_5.findChild(QPushButton, "state"+str(s)).setEnabled(False)
                self.ui.verticalLayoutWidget_5.findChild(QPushButton, "state"+str(s)).released.connect(partial(self.changeState,s))
                
    def sendControl(self, thrust, yawChange, pitchChange, rollChange):
        newYaw = self.constrainAngle(yawChange + self.Yaw, 360)
        newPitch = pitchChange
        newRoll = rollChange
        self.ui.Y.setText(str(newYaw))
        self.ui.P.setText(str(newPitch))
        self.ui.R.setText(str(newRoll))

        if self.State == VoConfig.MOVE:
            self.comms.writeCmd(VoConfig.MOVE, thrust, newYaw, newPitch, newRoll)
    
    # Create a "never ending" angle
    def constrainAngle(self, angle, constrain):
        if angle >= constrain:
            return constrainAngle(angle - constrain, constrain)
        return angle
    
    def controlAlert(self, data, isError):
        self.comms.writeToLog(data)
        if isError and self.comms.connected != 0:
            self.comms.ComDisconnect() # Disconnect if no joystick available
    
    def IPvalid(self):        
        # Use QHostAddress to check if address is valid:
        self.ui.connectBtn.setEnabled(
                QtNetwork.QHostAddress().setAddress(self.ui.vIP.text())
            )
    
    def changeState(self, s):
        if s == VoConfig.TEST:
            self.VoTest_i.show()
        elif s == VoConfig.CALIBRATE:
            pass
        else:
            self.comms.writeCmd(s)
    
    # STATUS FORMAT: ' UP_TIME ; STATE ; 100*q[0],100*q[1],100*q[2],100*q[3],HEADING,ALTITUDE ; FRONT_LEFT,FRONT_RIGHT,BACK_LEFT,BACK_RIGHT ; BAT_LVL '
    def newData(self, cmd, params):
        # if command is a STATUS, update local data
        if cmd == VoConfig.Q_STATUS:
            self.UpTime = params[0]
            self.State = params[1]
            self.q = [x / 100 for x in params[2:6]]
            self.Yaw,self.Pitch,self.Roll = [30, 20, 30]#[x for x in self.euler_from_quaternion(self.q, "szyx")] # Yaw = Z Axis, Pitch = Y Axis, Roll = X Axis
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
            if params[0] == VoConfig.Q_WOKE_UP:
                self.comms.writeToLog('ALERT: VoCopter woke up!')
        
        # if command is a SLEEPING alert, update local data.
        elif cmd == VoConfig.Q_SLEEPING:
            self.State = VoConfig.SLEEP
            self.updateUI()
        
        elif cmd == VoConfig.Q_TEST:
            if params[0] == VoConfig.Q_TEST_IMU:
                self.VoTest_i.RawValues(params)
            elif params[0] == VoConfig.Q_TEST_PID:
                self.VoTest_i.PID(params)
        else:
            self.comms.writeToLog("ERROR: Unknown command was received. Command ID: "+cmd)
    
    def updateUI(self):
        self.ui.batLvl.setProperty("value", self.BatLvl)
        self.ui.m1.setProperty("value", self.Thrusts[0] / 2.55)
        self.ui.m2.setProperty("value", self.Thrusts[1] / 2.55)
        self.ui.m3.setProperty("value", self.Thrusts[2] / 2.55)
        self.ui.m4.setProperty("value", self.Thrusts[3] / 2.55)
        self.ui.connectionTime.setText(str(self.UpTime))
        self.ui.alt.setText(str(self.Altitude))
        self.fixedQ = [self.q[i] + VoConfig.QOffset[i] for i in xrange(len(self.q))]
        self.VoCopter3D_i.rotateToQ(self.fixedQ)
        self.VoCopter3D_i.ren.ResetCamera()
        
        self.ui.head.setText(str(self.Heading))
        self.ui.yaw.setText(str(self.Yaw))
        self.ui.pitch.setText(str(self.Pitch))
        self.ui.roll.setText(str(self.Roll))
        
        # Enable / Disable state buttons:
        for s in xrange(VoConfig.NUM_OF_STATES):
            if s == self.State:
                self.ui.verticalLayoutWidget_5.findChild(QPushButton, "state"+str(s)).setEnabled(False)
            elif s != 4 and s != 6: # No button for state 4, state 6 not implemented yet
                self.ui.verticalLayoutWidget_5.findChild(QPushButton, "state"+str(s)).setEnabled(True)
    
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
    
    
    # epsilon for testing whether a number is close to zero
    _EPS = numpy.finfo(float).eps * 4.0

    # axis sequences for Euler angles
    _NEXT_AXIS = [1, 2, 0, 1]

    # map axes strings to/from tuples of inner axis, parity, repetition, frame
    _AXES2TUPLE = {
        'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
        'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
        'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
        'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
        'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
        'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
        'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
        'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}
    
    _TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())
    
    def euler_from_matrix(self, matrix, axes='sxyz'):
        """Return Euler angles from rotation matrix for specified axis sequence.

        axes : One of 24 axis sequences as string or encoded tuple

        Note that many Euler angle triplets can describe one matrix.

        >>> R0 = euler_matrix(1, 2, 3, 'syxz')
        >>> al, be, ga = euler_from_matrix(R0, 'syxz')
        >>> R1 = euler_matrix(al, be, ga, 'syxz')
        >>> numpy.allclose(R0, R1)
        True
        >>> angles = (4*math.pi) * (numpy.random.random(3) - 0.5)
        >>> for axes in self._AXES2TUPLE.keys():
        ...    R0 = euler_matrix(axes=axes, *angles)
        ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
        ...    if not numpy.allclose(R0, R1): print(axes, "failed")

        """
        try:
            firstaxis, parity, repetition, frame = self._AXES2TUPLE[axes.lower()]
        except (AttributeError, KeyError):
            self._TUPLE2AXES[axes]  # validation
            firstaxis, parity, repetition, frame = axes

        i = firstaxis
        j = self._NEXT_AXIS[i+parity]
        k = self._NEXT_AXIS[i-parity+1]

        M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:3, :3]
        if repetition:
            sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
            if sy > self._EPS:
                ax = math.atan2( M[i, j],  M[i, k])
                ay = math.atan2( sy,       M[i, i])
                az = math.atan2( M[j, i], -M[k, i])
            else:
                ax = math.atan2(-M[j, k],  M[j, j])
                ay = math.atan2( sy,       M[i, i])
                az = 0.0
        else:
            cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
            if cy > self._EPS:
                ax = math.atan2( M[k, j],  M[k, k])
                ay = math.atan2(-M[k, i],  cy)
                az = math.atan2( M[j, i],  M[i, i])
            else:
                ax = math.atan2(-M[j, k],  M[j, j])
                ay = math.atan2(-M[k, i],  cy)
                az = 0.0

        if parity:
            ax, ay, az = -ax, -ay, -az
        if frame:
            ax, az = az, ax
        return ax, ay, az


    def euler_from_quaternion(self, quaternion, axes='sxyz'):
        """Return Euler angles from quaternion for specified axis sequence.

        >>> angles = euler_from_quaternion([0.99810947, 0.06146124, 0, 0])
        >>> numpy.allclose(angles, [0.123, 0, 0])
        True

        """
        return self.euler_from_matrix(self.quaternion_matrix(quaternion), axes)
    
    def quaternion_matrix(self, quaternion):
        """Return homogeneous rotation matrix from quaternion.

        >>> M = quaternion_matrix([0.99810947, 0.06146124, 0, 0])
        >>> numpy.allclose(M, rotation_matrix(0.123, [1, 0, 0]))
        True
        >>> M = quaternion_matrix([1, 0, 0, 0])
        >>> numpy.allclose(M, numpy.identity(4))
        True
        >>> M = quaternion_matrix([0, 1, 0, 0])
        >>> numpy.allclose(M, numpy.diag([1, -1, -1, 1]))
        True

        """
        q = numpy.array(quaternion, dtype=numpy.float64, copy=True)
        n = numpy.dot(q, q)
        if n < self._EPS:
            return numpy.identity(4)
        q *= math.sqrt(2.0 / n)
        q = numpy.outer(q, q)
        return numpy.array([
            [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
            [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
            [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
            [                0.0,                 0.0,                 0.0, 1.0]])

if __name__ == "__main__":
    QtGui.QApplication.setStyle("plastique")
    app = QtGui.QApplication(sys.argv)
    VoPilot_i = VoPilot()
    
    # Handle close event to correctly terminate the control thread
    def exitHandler():
        VoPilot_i.controlAlive = False
    
    app.aboutToQuit.connect(exitHandler)    
    VoPilot_i.show()
    sys.exit(app.exec_())