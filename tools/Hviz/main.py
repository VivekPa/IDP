from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QDialog, QMainWindow, QMessageBox
# from PyQt5.uic import loadUi
import mainWindow
import sys
import requests

from mainWindow import Ui_MainWindow

class Window(QMainWindow, Ui_MainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        # self.connectSignalsSlots()
    
    # def connectSignalsSlots(self):
    #     self.action_Exit.triggered.connect(self.close)
    #     self.action_Find_Replace.triggered.connect(self.findAndReplace)
    #     self.action_About.triggered.connect(self.about)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    myapp = Window()
    myapp.show()
    sys.exit(app.exec_())