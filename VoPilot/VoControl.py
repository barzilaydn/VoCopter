import os, sys, time
from PySide import QtGui, QtCore, QtNetwork
from PySide.QtGui import *
from PySide.QtCore import *
import pygame
import VoConfig

class VoControl(QtCore.QThread):
    def __init__(self,parent=None):  
        super(VoControl,self).__init__(parent)
        self.parent = parent
        self.signals = controlSignals()
        self.joystick = None
        pygame.init()
        
    def run(self):
        self.checkJoystick(True)
        while self.parent.controlAlive: # Keep reading joysticks and updating the copter. 
            
            while not self.checkJoystick(): time.sleep(2) # Wait till joystick connected
            
            if pygame.joystick.get_count() > 0:
                pygame.event.pump()
                for event in pygame.event.get(): # User did something
                    if event.type == pygame.JOYAXISMOTION: # He moved the Joystick.
                        yaw   = self.deadBand(self.joystick.get_axis(3)) * VoConfig.MAX_YAW    # Z Axis
                        pitch = -self.deadBand(self.joystick.get_axis(1)) * VoConfig.MAX_PITCH  # Y Axis
                        roll  = self.deadBand(self.joystick.get_axis(0)) * VoConfig.MAX_ROLL   # X Axis
                        baseThrust = 255 * (self.joystick.get_axis(2) + 1) / 2  # Throttle Axis
                        self.signals.cmdAvailable.emit(baseThrust, yaw, pitch, roll)
            else:
                self.signals.alert.emit("Please connect a joystick!", True)
    
    def deadBand(self, joy):
        if abs(joy) < VoConfig.JOYSTICK_DEADBAND:
            return 0
        return (abs(joy)/joy)*joy*joy
    
    def checkJoystick(self, first=False):
        if self.joystick == None:
            pygame.joystick.quit()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:        
                # Initialize the joystick
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                print(self.joystick.get_numaxes())
                self.signals.alert.emit("Joystick connected: {}".format(self.joystick.get_name()), False)        
                return True
        else:
            return True
        
        if first:
            self.signals.alert.emit("Please connect a joystick!", True)
        return False

class controlSignals(QObject):
    # Set up a Signal to indicate new data:
    cmdAvailable = Signal(int, int, int, int)
    # Set up a Signal to indicate alert: (DATA, isError)
    alert = Signal(str, bool)