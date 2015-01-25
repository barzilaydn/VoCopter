#!/usr/bin/env python
 
import sys
import vtk
from PySide import QtCore, QtGui
from vtk.qt4.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
 
class VoCopter3D(QtGui.QMainWindow):
    def __init__(self, filename, parent = None):
        QtGui.QMainWindow.__init__(self, parent)
 
        self.frame = QtGui.QFrame()
 
        self.vl = QtGui.QVBoxLayout()
        self.vtkWidget = QVTKRenderWindowInteractor(self.frame)
        self.vl.addWidget(self.vtkWidget)
 
        self.ren = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.ren)
        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()
        self.vtkWidget.RemoveObservers('LeftButtonPressEvent')
        self.vtkWidget.RemoveObservers('RightButtonPressEvent')
        
        reader = vtk.vtkSTLReader()
        reader.SetFileName(filename)
        
        # create a transform that rotates the cone
        self.transform = vtk.vtkTransform()
        self.transform.RotateWXYZ(0,0,0,0)
        self.transformFilter=vtk.vtkTransformPolyDataFilter()
        self.transformFilter.SetTransform(self.transform)
        self.transformFilter.SetInputConnection(reader.GetOutputPort())
        self.transformFilter.Update()
        
        mapper = vtk.vtkPolyDataMapper()
        if vtk.VTK_MAJOR_VERSION <= 5:
            mapper.SetInput(self.transformFilter.GetOutput())
        else:
            mapper.SetInputConnection(self.transformFilter.GetOutputPort())
         
        # Create an actor
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.SetMapper(mapper)
 
        self.ren.AddActor(actor)
 
        self.ren.ResetCamera()
 
        self.frame.setLayout(self.vl)
        self.setCentralWidget(self.frame)
 
        self.show()
        self.iren.Initialize()
    
    def rotateToQ(self, q):
        self.transform.RotateWXYZ(q[0],q[1],q[2],q[3])
        self.transformFilter.SetTransform(self.transform)
        self.transformFilter.Update()
