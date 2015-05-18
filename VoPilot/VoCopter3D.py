#!/usr/bin/env python

import vtk
from PySide import QtGui
from vtk.qt4.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor


class VoCopter3D(QtGui.QMainWindow):
    def __init__(self, filename, parent=None):
        QtGui.QMainWindow.__init__(self, parent)
        self.frame = QtGui.QFrame()

        self.vl = QtGui.QVBoxLayout()
        self.vtkWidget = QVTKRenderWindowInteractor(self.frame)
        self.vl.addWidget(self.vtkWidget)

        self.ren = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.ren)
        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()
        self.vtkWidget.RemoveAllObservers()

        reader = vtk.vtkSTLReader()
        reader.SetFileName(filename)
        reader.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(reader.GetOutputPort())

        # Create an actor
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(mapper)
        self.actor.GetProperty().SetColor(0.39, 0.27, 0.58)  # (R,G,B)
        self.actor.SetOrigin((0, 0, 0))
        self.actor.RotateX(185)
        self.actor.RotateY(-4)
        self.actor.RotateZ(4)
        self.ren.AddActor(self.actor)
        self.ren.SetBackground(0.94, 0.94, 0.94)
        self.ren.ResetCamera()
        camera = self.ren.GetActiveCamera()
        camera.Zoom(1.1)

        self.frame.setLayout(self.vl)
        self.setCentralWidget(self.frame)

        self.show()
        self.iren.Initialize()

    def rotateToYPR(self, y, p, r):
        self.actor.SetOrigin((0, 0, 0))
        oriantation = self.actor.GetOrientation()
        oriantation = [-ori for ori in oriantation]
        self.actor.SetOrientation(oriantation)
        self.actor.SetOrigin((0, 0, 0))
        self.actor.SetOrientation((185-p, -4.5-y, -3-r))
        self.ren.ResetCamera()
        camera = self.ren.GetActiveCamera()
        camera.Zoom(1.5)
        self.ren.Modified()
        self.ren.Render()
        self.iren.Initialize()