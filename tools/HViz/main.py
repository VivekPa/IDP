import mainWindow
from mainWindow import Ui_MainWindow

import sys
import requests
import numpy as np

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import (
    QApplication, QDialog, QMainWindow, QMessageBox, QGraphicsView, 
    QGraphicsRectItem, QGraphicsEllipseItem, QGraphicsScene, QGraphicsItem
    )
from PyQt5.QtGui import QPen, QPainter, QBrush
from PyQt5.QtCore import Qt

import pyqtgraph as pg
from pyqtgraph import ROI, PlotWidget

import utils.click as click

pg.setConfigOption('background', None)                 # set the background transparent (by default pyqtgraph is in darkmode)
pg.setConfigOption('foreground', 'k')

class Window(QMainWindow, Ui_MainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowTitle('HViz')

        """Initialise Variables"""
        self.p              = self.ui.MapPlotWidget         # PlotWidget
        self.arenawidth     = 240                           # set Arena Width
        self.homewidth      = 50                            # set Arena Width

        self.blue_home      = np.array([1,-1])              # set blue home coordinates
        self.blue_waypoints = np.array(self.blue_home)      # initialise blue waypoints
        self.blue_blocks    = np.array([])                  # initialise blue blocks numpy array

        self.red_home       = np.array([1,1])               # set red home coordinates
        self.red_waypoints  = np.array(self.red_home)       # initialise red waypoints
        self.red_blocks     = np.array([])                  # initialise red blocks numpy array
        
        """Initialise Objects"""
        self.movingRect     = MovingRect(0,0,10)            # initialise MovingRect object
        self.movingCircle   = MovingCircle(0,0,5)           # initialise MovingCircle object
        
        """Initialise Functions"""
        self.plotArena()                                    # Plot the Arena
        # self.drawCoordinateReference()

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

    def plotArena(self):
        arenawidth  = self.arenawidth
        homewidth   = self.homewidth

        self.p.showGrid(x = True, y = True, alpha = 0.3)
        
        arena       = self.movingRect
        arena       = MovingRect(-arenawidth/2, -arenawidth/2, arenawidth)
        arena.setPen(pg.mkPen('k', width = 5))
        # arena.setBrush(Qt.transparent)
        arena.setBrush(Qt.lightGray)
        self.p.addItem(arena)
        
        redHome     = self.movingRect
        redHome     = MovingRect(-arenawidth/2, arenawidth/2-homewidth, homewidth)
        # redHome.setPen(pg.mkPen('k', width = 1))
        redHome.setBrush(Qt.darkRed)
        self.p.addItem(redHome)

        blueHome     = self.movingRect
        blueHome     = MovingRect(-arenawidth/2, -arenawidth/2, homewidth)
        # blueHome.setPen(pg.mkPen('k', width = 1))
        blueHome.setBrush(Qt.darkBlue)
        self.p.addItem(blueHome)

        # p.addItem(home_red)
        # p.addItem(home_blue)

        # p.plot([-1,0,0.5,1], [-1,0.5,0,1], pen = None, symbolPen='r', symbolBrush=(151,21,0), symbol='h')

        # p.setLabels(bottom='X', left='Z')
        self.p.setXRange(-arenawidth/2, arenawidth/2, padding=0.05)
        self.p.setYRange(-arenawidth/2, arenawidth/2, padding=0.05)
        # p.getPlotItem().hideAxis('bottom')
        # p.getPlotItem().hideAxis('left')

    def addRedBlock(self, coordinates, movingRect):
        x = coordinates[0]
        z = coordinates[1]
        movingRect = MovingRect(x, z)
        movingRect.setBrush(Qt.red)
        self.p.addItem(movingRect)
    
    def addBlueBlock(self, coordinates, movingRect):
        x = coordinates[0]
        z = coordinates[1]
        movingRect = MovingRect(x, z)
        movingRect.setBrush(Qt.blue)
        self.p.addItem(movingRect)

    # def drawCoordinateReference(self):
    #     reference   = self.QGraphicsLineItem
    #     reference.begin(self)
    #     # reference.setRenderHint(QPainter.Antialiasing)
    #     reference.setPen(Qt.red)
    #     reference.setBrush(Qt.red)
    #     reference.drawLine(400, 100, 100, 100)
    #     reference.drawLine(150, 150, 100, 100)
    #     reference.drawLine(150, 50, 100, 100)

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

class MovingRect(QGraphicsRectItem):
    def __init__ (self, x, y, w=10):
        super().__init__(0, 0, w, w)
        self.setPos(x, y)
        self.setBrush(Qt.blue)
        self.setPen(pg.mkPen('k', width=2))
        self.setAcceptHoverEvents(True)

class MovingCircle(QGraphicsEllipseItem):
    def __init__ (self, x, y, r):
        super().__init__(0, 0, r, r)
        self.setPos(x, y)
        self.setBrush(Qt.blue)
        self.setPen(pg.mkPen('k', width=2))
        self.setAcceptHoverEvents(True)




if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = Window()
    
    rCoordinates = np.array([[-84,60],[80,30],[10,-30]])
    bCoordinates = np.array([[-50,20],[-100,-100],[20,90]])

    block = win.movingRect
    
    for i in range(len(rCoordinates)):
        rCoordinate_i = rCoordinates[i]
        win.addRedBlock(rCoordinate_i, block)
    
    for i in range(len(bCoordinates)):
        bCoordinate_i = bCoordinates[i]
        win.addBlueBlock(bCoordinate_i, block)
    
    win.show()
    sys.exit(app.exec_())