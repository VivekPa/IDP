"""
Code from: https://github.com/abhilb/pyqtgraphutils
Abhilash Babu
"""

from PyQt5 import QtGui, QtCore
import pyqtgraph as pg
import numpy as np


class LineSegmentItem(pg.GraphicsObject):
    def __init__(self, p1, p2):
        pg.GraphicsObject.__init__(self)
        self.p1 = p1
        self.p2 = p2
        self.generatePicture()

    def generatePicture(self):
        self.picture = QtGui.QPicture()
        p = QtGui.QPainter(self.picture)
        p.setPen(pg.mkPen('w'))
        p.drawLine(QtCore.QPoint(self.p1[0], self.p1[1]), QtCore.QPoint(self.p2[0], self.p2[1]))
        p.end()

    def paint(self, p, *args):
        p.drawPicture(0, 0, self.picture)

    def boundingRect(self):
        return QtCore.QRectF(self.picture.boundingRect())


class CircleItem(pg.GraphicsObject):
    def __init__(self, center, radius, color='k', width=2, style=QtCore.Qt.SolidLine, fill=False):
        pg.GraphicsObject.__init__(self)
        self.center = center
        self.radius = radius
        self.color = color
        self.width = width
        self.style = style
        self.fill = fill
        self.generatePicture()

    def generatePicture(self):
        """
        Unfortunately the drawEllipse() function in QPainter only accepts integer coordinate values.
        """
        # center = np.array(self.center) - np.array([self.radius,self.radius])
        # print(center)
        self.picture = QtGui.QPicture()
        p = QtGui.QPainter(self.picture)
        if self.fill == True:
            p.setBrush(pg.mkBrush(self.color, width=self.width))
        p.setPen(pg.mkPen(self.color, width=self.width, style=self.style))
        p.drawEllipse(self.center[0], self.center[1], self.radius * 2.0, self.radius * 2.0)
        p.end()

    def paint(self, p, *args):
        p.drawPicture(0, 0, self.picture)

    def boundingRect(self):
        return QtCore.QRectF(self.picture.boundingRect())


class RectangleItem(pg.GraphicsObject):
    def __init__(self, topLeft, size, color, width=2, style=QtCore.Qt.SolidLine, fill=False):
        pg.GraphicsObject.__init__(self)
        self.topLeft = topLeft
        self.size = size
        self.color = color
        self.width = width
        self.style = style
        self.fill = fill
        self.generatePicture()

    def generatePicture(self):
        """
        Unfortunately the drawRect() function in QPainter only accepts integer coordinate values.
        """
        self.picture = QtGui.QPicture()
        p = QtGui.QPainter(self.picture)
        if self.fill == True:
            p.setBrush(pg.mkBrush(self.color, width=self.width))
        p.setPen(pg.mkPen(self.color, width=self.width, style=self.style))
        tl = QtCore.QPointF(self.topLeft[0], self.topLeft[1])
        size = QtCore.QSizeF(self.size[0], self.size[1])
        p.drawRect(QtCore.QRectF(tl, size))
        p.end()

    def paint(self, p, *args):
        p.drawPicture(0, 0, self.picture)

    def boundingRect(self):
        return QtCore.QRectF(self.picture.boundingRect())