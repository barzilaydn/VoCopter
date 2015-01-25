# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'VoPilot_Console.ui'
#
# Created: Thu Dec 25 15:56:06 2014
#      by: pyside-uic 0.2.15 running on PySide 1.2.2
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_Console(object):
    def setupUi(self, Console):
        Console.setObjectName("Console")
        Console.resize(832, 517)
        Console.setMinimumSize(QtCore.QSize(832, 517))
        Console.setMaximumSize(QtCore.QSize(832, 517))
        Console.setLocale(QtCore.QLocale(QtCore.QLocale.English, QtCore.QLocale.UnitedStates))
        self.verticalLayoutWidget = QtGui.QWidget(Console)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 10, 811, 491))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtGui.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        font = QtGui.QFont()
        font.setPointSize(10)
        self.comViewer = QtGui.QTextEdit(self.verticalLayoutWidget)
        self.comViewer.setObjectName("comViewer")
        self.comViewer.setReadOnly(True)
        self.comViewer.setFont(font)
        self.verticalLayout.addWidget(self.comViewer)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.cmd = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.cmd.setObjectName("cmd")
        self.horizontalLayout.addWidget(self.cmd)
        self.sendCmd = QtGui.QPushButton(self.verticalLayoutWidget)
        self.sendCmd.setObjectName("sendCmd")
        self.horizontalLayout.addWidget(self.sendCmd)
        self.saveLog = QtGui.QPushButton(self.verticalLayoutWidget)
        self.saveLog.setObjectName("saveLog")
        self.horizontalLayout.addWidget(self.saveLog)
        self.verticalLayout.addLayout(self.horizontalLayout)

        self.retranslateUi(Console)
        QtCore.QMetaObject.connectSlotsByName(Console)

    def retranslateUi(self, Console):
        Console.setWindowTitle(QtGui.QApplication.translate("Console", "VoPilot - Console", None, QtGui.QApplication.UnicodeUTF8))
        self.sendCmd.setText(QtGui.QApplication.translate("Console", "Send", None, QtGui.QApplication.UnicodeUTF8))
        self.saveLog.setText(QtGui.QApplication.translate("Console", "Save Log", None, QtGui.QApplication.UnicodeUTF8))

