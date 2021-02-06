import mainWindow
import sys
import requests
from mainWindow import Ui_MainWindow

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QDialog, QMainWindow, QMessageBox, QOpenGLWidget
from PyQt5.QtGui import QPen
from PyQt5.QtCore import Qt

# import OpenGL.GL as gl
# from OpenGL import GLU
# import utils.glutils as glutils

import pyqtgraph as pg
from pyqtgraph import ROI

import utils.customExporter

import utils.pyqtgraphutils as pgutils
import numpy as np

# import 

import utils.click as click

# pg.setConfigOption('background', "#708090")
pg.setConfigOption('background', None)          # set the background transparent
pg.setConfigOption('foreground', 'k')

# from qwt import QwtPlot, QwtPlotMarker, QwtLegend, QwtPlotCurve, QwtText
# from PyQt5.uic import loadUi

# import matplotlib
# matplotlib.use('Qt5Agg')
# from matplotlib.figure import Figure

class Window(QMainWindow, Ui_MainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowTitle('HViz')

        """Define Variables"""
        self.p = self.ui.MapPlotWidget         # PlotWidget
        self.arenawidth  = 2.4
        self.homewidth   = 1.6

        self.blue_home = np.array([1,0,1])
        self.blue_waypoints = np.array(self.blue_home)

        self.red_home = np.array([1,0,-1])
        self.red_waypoints = np.array(self.red_home)

        """Call Functions"""
        self.plot()
        self.plot2img()
        # self.graphics()

        # timer = QtCore.QTimer(self)
        # timer.setInterval(20)           # period, in milliseconds
        # # timer.timeout.connect(GLWidget.updateGL)
        # timer.start()

        # self.connectSignalsSlots()

    # def connectSignalsSlots(self):
    #     # self.action_Exit.triggered.connect(self.close)
    #     # self.action_Find_Replace.triggered.connect(self.findAndReplace)
    #     # self.action_About.triggered.connect(self.about)

    def plot(self):
        arenawidth  = self.arenawidth
        homewidth   = self.homewidth

        p = self.p
        p.showGrid(x = True, y = True, alpha = 0.3)

        red         = pgutils.RectangleItem([-arenawidth/2, -arenawidth/2],[-arenawidth/2+homewidth, -arenawidth/2+homewidth],
                                         'r', width=0.1, fill=True)
        blue        = pgutils.RectangleItem([-arenawidth/2, arenawidth/2],[-arenawidth/2+homewidth, arenawidth/2-homewidth],
                                         'b', width=0.1, fill=True)
        arena       = pgutils.RectangleItem([-arenawidth/2, -arenawidth/2],[+arenawidth, +arenawidth],
                                         'k', width=5, fill=False)
        # home_red    = pgutils.CircleItem([-arenawidth/2,-arenawidth/2], 0.5, 'k', style=QtCore.Qt.DashLine)
        # home_blue   = pgutils.CircleItem([-1.7, -1.7], 0.5, 'k', style=QtCore.Qt.DashLine)

        p.addItem(red)
        p.addItem(blue)
        p.addItem(arena)
        # p.addItem(home_red)
        # p.addItem(home_blue)

        # p.plot([-1,0,0.5,1], [-1,0.5,0,1], pen = None, symbolPen='r', symbolBrush=(151,21,0), symbol='h')

        # p.setLabels(bottom='X', left='Z')
        p.setXRange(-arenawidth/2, arenawidth/2, padding=0.05)
        p.setYRange(-arenawidth/2, arenawidth/2, padding=0.05)
        p.getPlotItem().hideAxis('bottom')
        p.getPlotItem().hideAxis('left')
        # pg.ViewBox(lockAspect=True, enableMouse=False)

    def plot2img(self):
        p           = self.p
        arenawidth  = self.arenawidth

        img = pg.ImageItem()
        p.addItem(img)

        exporter = utils.customExporter.PQG_ImageExporter(p.plotItem)
        exporter.parameters()['width'] = self.p.width()
        exporter.parameters()['height'] = self.p.height()
        # exporter.parameters()['width'] = 100
        # exporter.parameters()['height'] = 100
        exporter.export('test.png')

        roi = pg.ROI(pos=[-arenawidth/2, -arenawidth/2], size=[arenawidth, arenawidth])
        # roi.addScaleHandle(pos=[0.5, 1], center=[0.0, 0.0]) # This allows you to drag and change the ROI manually
        # roi.addScaleHandle(pos=[0, 0.5], center=[0.0, 0.0])
        p.addItem(roi)
        # roi.setPen(pg.mkPen(color='k', width=1, style=QtCore.Qt.DashLine)) # This allows you to see the ROI
        
        # img.scale(0.2, 0.2)
        # img.translate(-50, 0)

    # def updatePlot():
    #     global img, roi, data, p
    #     selected = roi.getArrayRegion(data.img)
    #     p.plot(selected.mean(axis=0), clear=True)


    # def graphics(self):
        # g           = self.ui.GLWidget
        # g.initializeGL()
        
    # def onclick(self):
    #     p = self.p
    #     items = p.scene().items()

    def removeWaypoint(self):
        ...
    def removePath(self):
        ...
    
    def export_path(self):
        ...

    def live_preview(self):
        ...

    def newFile(self):
        ...
    def openFile(self):
        ...
    def importFile(self):
        ...
    def exportFile(self):
        ...
   
    def showRed(self):
        ...
    def showBlue(self):
        ...
    
    def change2white(self):
        ...
    def change2slategray(self):
        ...
    def change2charcoal(self):
        ...



if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = Window()
    win.show()
    sys.exit(app.exec_())