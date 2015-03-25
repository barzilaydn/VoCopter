#!/usr/bin/python
# encoding: utf-8

import time
import struct
from PySide import QtGui, QtCore, QtNetwork
from PySide.QtCore import *
from PySide.QtGui import *
from VoConsole_ui import Ui_Console
import VoConfig

class VoConsole(QMainWindow):
    cmd = 0
    params = []
    
    def __init__(self, parent=None):
        super(VoConsole, self).__init__(parent)
        
        self.parent = parent
        
        self.ui = Ui_Console()
        self.ui.setupUi(self)
        
        self.ui.saveLog.released.connect(self.saveLog)
        self.ui.sendCmd.released.connect(self.sendCmd)
        self.ui.sendCmd.setEnabled(parent.comms.connected)             
        self.ui.cmd.setEnabled(parent.comms.connected)             
        self.ui.cmd.setPlaceholderText("Syntax: CMD;PARAM;PARAM;PARAM;...")
        self.ui.cmd.textChanged.connect(self.validateCmd)
    
    def saveLog(self):
        # Only open dialog if there is no filename yet
        self.filename = QtGui.QFileDialog.getSaveFileName(self, 'Save Log', filter="*.txt")[0]
        
        # Append extension if not there yet
        if not self.filename.endswith(".txt"):
            self.filename += ".txt"

        # We just store the contents of the text file along with the
        # format in html, which Qt does in a very nice way for us
        with open(self.filename,"wt") as file:
            file.write(self.ui.comViewer.toPlainText())
    
    def validateCmd(self):
        try:
            rawCmd = self.ui.cmd.text()
            parsedCmd = rawCmd.split(';')
            flag = False
            
            cmd = int(parsedCmd[0])
            # At least one parameter + Valid state
            if len(parsedCmd) > 1 and \
                all([x != "" and (int(x) or x == "0") for x in parsedCmd]) and \
                cmd < VoConfig.NUM_OF_STATES and \
                cmd > 0:
                    flag = True
                    self.cmd = cmd
                    self.params = [int(x) for x in parsedCmd[1:]]
                    
            self.ui.sendCmd.setEnabled(flag)
        except ValueError:
            pass
    
    def logChange(self, data):
        self.ui.comViewer.append(data)
        # Auto-scroll to end:
        self.ui.comViewer.verticalScrollBar().setValue(self.ui.comViewer.verticalScrollBar().maximum())
    
    def sendCmd(self):
        self.parent.comms.writeCmd(self.cmd, *self.params) # Send the command, send params list as *args

class VoComms():
    
    def __init__(self, parent=None):
        
        self.parent = parent
        self.signals = commsSignals()
        
        # Set up the TCP socket:
        self.tcpSocket = QtNetwork.QTcpSocket()
        self.tcpSocket.setSocketOption(QtNetwork.QAbstractSocket.LowDelayOption, 1) # TCP NO_DELAY
        self.tcpSocket.setSocketOption(QtNetwork.QAbstractSocket.KeepAliveOption, 1) # TCP KEEP_ALIVE
        self.tcpSocket.readyRead.connect(self.readCmd)
        self.tcpSocket.error.connect(self.handleError)
        self.tcpSocket.connected.connect(self.ComConnected)
        self.tcpSocket.disconnected.connect(self.ComDisconnected)
        self.connected = 0
        self.goodConnection = False
        
    def toggleConnection(self):
        if self.connected == 0:
            self.ComConnect()
        else:
            self.ComDisconnect()
    
    def ComConnect(self):
        self.writeToLog("Trying to connect to: {}:{}".format(self.parent.ui.vIP.text(), VoConfig.PORT))
        self.connected = 2 # Connecting..
        self.signals.connectionChange.emit(self.connected)
        self.tcpSocket.connectToHost(self.parent.ui.vIP.text(), VoConfig.PORT)
    
    def ComConnected(self):
        self.connected = 1
        self.writeToLog("Connected to a VoCopter unit.")
        self.signals.connectionChange.emit(self.connected)
    
    def ComDisconnect(self):
        self.connected = 3 # Disconnecting..
        self.signals.connectionChange.emit(self.connected)
        self.tcpSocket.disconnectFromHost()
    
    def ComDisconnected(self):
        self.connected = 0
        self.writeToLog("Disconnected from the VoCopter unit.")
        self.signals.connectionChange.emit(self.connected)
    
    def handleError(self, socketError):
        if socketError == QtNetwork.QAbstractSocket.RemoteHostClosedError:
            self.writeToLog("ERROR: The VoCopter closed the connection.")
        elif socketError == QtNetwork.QAbstractSocket.HostNotFoundError:
            self.writeToLog("ERROR: The VoCopter was not found, disconnecting. "
                            "Please check the IP address setting and verify that the unit is turned on.")
        elif socketError == QtNetwork.QAbstractSocket.ConnectionRefusedError:
            self.writeToLog("ERROR: The VoCopter refused the connection, disconnecting. Please try restarting the VoCopter.")
        else:
            self.writeToLog("ERROR: The following error occurred: %s, disconnecting." % self.tcpSocket.errorString())
        
        # Disconnect when faced with error:
        self.ComDisconnect()    
    
    def readCmd(self):        
        if self.tcpSocket.bytesAvailable() >= VoConfig.COMMAND_SIZE:            
            data = QtCore.QByteArray()
            head = QtCore.QByteArray()
            tail = QtCore.QByteArray()
            
            head = self.tcpSocket.read(1)
            
            while head[0] != QtCore.QByteArray.fromRawData(VoConfig.CMD_HEAD):
                if self.tcpSocket.bytesAvailable() >= VoConfig.COMMAND_SIZE:
                    head = self.tcpSocket.read(1)
                else:
                    self.writeToLog("Received bad head.")
                    return
            
            data = self.tcpSocket.read(VoConfig.COMMAND_SIZE - 1)            
            params = [] # List of received params.
            tail += data[VoConfig.COMMAND_SIZE-2]
            
            # Read command only if it is valid:
            if tail == QtCore.QByteArray.fromRawData(VoConfig.CMD_TAIL):
                #CMD
                stream = QtCore.QDataStream(data[0:4])
                stream.setByteOrder(QtCore.QDataStream.LittleEndian)
                cmd = stream.readInt32()
                
                #PARAMS
                for i in range(VoConfig.MAX_CNTRL_PARAMS):
                    stream = QtCore.QDataStream(data[4+i*4:6+(i+1)*4])
                    stream.setByteOrder(QtCore.QDataStream.LittleEndian)
                    params.append(stream.readInt32())
                
                if cmd != VoConfig.Q_ACK and cmd != VoConfig.Q_RECV and cmd != VoConfig.Q_STATUS:
                    self.writeToLog("Received command: {} , Params are: {}".format(cmd, params))
                self.signals.cmdAvailable.emit(cmd, params)       
    
    def writeCmd(self, cmd, *argv):
        if self.connected == 2 or self.connected == 1:
            if self.goodConnection:
                self.outputCmd(cmd, *argv)  
            else:
                self.writeToLog("Starting VoCopter command control. Orig CMD: {}".format(cmd))
                self.outputCmd(VoConfig.Q_SYN, 0) # Wake it up..      
        else:
            self.writeToLog("ERROR: Command not sent, VoCopter is not connected.")
    
    def outputCmd(self, cmd, *params):        
        count = 0
        buffer = ['\x00']*VoConfig.COMMAND_SIZE
        
        # Add head:
        buffer[0] = VoConfig.CMD_HEAD
        
        # Add CMD in Little-endian:
        self.insertToBuffer(buffer, struct.pack("<i", cmd), 1)
        
        # Add arguments in Little-endian:
        for arg in params:
            if count < VoConfig.MAX_CNTRL_PARAMS:
                self.insertToBuffer(buffer, struct.pack("<i", arg), 5 + 4 * count)
                count = count + 1
        
        # Fill up rest of buffer:
        while count < VoConfig.MAX_CNTRL_PARAMS:
            self.insertToBuffer(buffer, struct.pack("<i", 0), 5 + 4 * count)
            count = count + 1
        
        # Add tail:
        self.insertToBuffer(buffer, VoConfig.CMD_TAIL, VoConfig.COMMAND_SIZE-1)
        
        str = ''
        for b in buffer:
            str += b
        data = QtCore.QByteArray.fromRawData(str)
        
        # Make sure all previous data was sent:
        self.tcpSocket.flush()
        
        # Send the data:
        self.tcpSocket.write(data)
        
        if cmd < VoConfig.NUM_OF_STATES:
            self.writeToLog("Sending command: {} , Params are: {}".format(cmd, params))
    
    def insertToBuffer(self, buffer, data, startingIndex):
        for i in xrange(0, len(data)):
            if startingIndex + i < len(buffer):
                buffer[startingIndex + i] = data[i]
    
    def writeToLog(self, data):
        msg = "[{}] {}".format(time.strftime('%X'), data)
        self.parent.ui.statusbar.showMessage(msg)
        self.signals.logChange.emit(msg)

class commsSignals(QObject):
    # Set up a Signal to indicate new data:
    cmdAvailable = Signal(int, list)
    # Set up a Signal to indicate new connection:
    connectionChange = Signal(int)
    # Set up a Signal to indicate change in log:
    logChange = Signal(str)