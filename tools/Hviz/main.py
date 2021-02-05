import mainWindow
import sys
import requests
from mainWindow import Ui_MainWindow

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QDialog, QMainWindow, QMessageBox
from PyQt5.QtGui import QPen
from PyQt5.QtCore import Qt

import pyqtgraph as pg
import pyqtgraphutils as pgutils
import numpy as np

# pg.setConfigOption('background', "#708090")
pg.setConfigOption('background', 'w')
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
        self.plot()

        # self.connectSignalsSlots()

    # def connectSignalsSlots(self):
    #     self.action_Exit.triggered.connect(self.close)
    #     self.action_Find_Replace.triggered.connect(self.findAndReplace)
    #     self.action_About.triggered.connect(self.about)

    def plot(self):
        # x = np.arange        self.drawRect()

        arenawidth  = 2.4
        homewidth   = 1.6

        p = self.ui.graphicsView                   # pg.PlotWidget
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

        p.setXRange(-arenawidth/2, arenawidth/2, padding=0.05)
        p.setYRange(-arenawidth/2, arenawidth/2, padding=0.05)
        # p.plot(x, y, pen=pg.mkPen(color='r', width=2))
        # pg.ViewBox(parent=None, lockAspect=True, enableMouse=False)


        # p.plot(pen=pg.mkPen(color='r', width=2))
        # p.drawLine(QtCore.QPointF(0, 0), QtCore.QPointF(1, 1))
        # p.setBrush(pg.mkBrush('r'))
        # p.drawRect(QtCore.QRectF(0, 0, 4.5, 4.5))
        # p.end()
        # self.graphWidget_2D.addItem(QtCore.QRectF(self.picture.boundingRect()))
    
    # def drawRect(self):
    #     self.ui.graphicsView = QtGui.QPicture()
    #     p = QtGui.QPainter(self.ui.graphicsView)
    #     p.setPen(pg.mkPen('w'))
    #     p.drawLine(QtCore.QPointF(0, 0), QtCore.QPointF(1, 1))
    #     p.setBrush(pg.mkBrush('g'))
    #     p.drawRect(QtCore.QRectF(0, 0, 4.5, 4.5))
    #     p.end()
    #     self.graphWidget_2D.addItem(QtCore.QRectF(self.picture.boundingRect()))


    def newFile(self):
        ...
    def openFile(self):
        ...
    def importFile(self):
        ...
    def exportFile(self):
        ...

    def addWaypoint(self):
        ...
    def removeWaypoint(self):
        ...
    def removePath(self):
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