import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np

def imageHoverEvent(event, img):
    pos = event.pos()
    # i, j = pos.y(),pos.x()
    # i = int(np.clip(i, 0, data.shape[0] - 1))
    # j = int(np.clip(j, 0, data.shape[1] - 1))
    # val = data[i, j]
    ppos = img.mapToParent(pos)
    x, y = ppos.x(), ppos.y()

    return x, y