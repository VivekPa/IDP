# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'HViz.ui'
#
# Created by: PyQt5 UI code generator 5.15.2
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets
from pyqtgraph import PlotWidget

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1020, 700)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        MainWindow.setMaximumSize(QtCore.QSize(1920, 1080))
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.centralwidget.sizePolicy().hasHeightForWidth())
        self.centralwidget.setSizePolicy(sizePolicy)
        self.centralwidget.setObjectName("centralwidget")
        self.layoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget.setGeometry(QtCore.QRect(20, 20, 982, 602))
        self.layoutWidget.setObjectName("layoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.layoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.MapPlotWidget = PlotWidget(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.MapPlotWidget.sizePolicy().hasHeightForWidth())
        self.MapPlotWidget.setSizePolicy(sizePolicy)
        self.MapPlotWidget.setMinimumSize(QtCore.QSize(600, 600))
        self.MapPlotWidget.setMaximumSize(QtCore.QSize(900, 900))
        self.MapPlotWidget.viewport().setProperty("cursor", QtGui.QCursor(QtCore.Qt.CrossCursor))
        self.MapPlotWidget.setMouseTracking(True)
        self.MapPlotWidget.setObjectName("MapPlotWidget")
        self.horizontalLayout.addWidget(self.MapPlotWidget)
        spacerItem = QtWidgets.QSpacerItem(10, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.red_verticalLayout = QtWidgets.QVBoxLayout()
        self.red_verticalLayout.setContentsMargins(-1, -1, -1, 0)
        self.red_verticalLayout.setObjectName("red_verticalLayout")
        self.red_exportButton = QtWidgets.QPushButton(self.layoutWidget)
        self.red_exportButton.setObjectName("red_exportButton")
        self.red_verticalLayout.addWidget(self.red_exportButton)
        self.red_realtimeButton = QtWidgets.QPushButton(self.layoutWidget)
        self.red_realtimeButton.setObjectName("red_realtimeButton")
        self.red_verticalLayout.addWidget(self.red_realtimeButton)
        self.red_compassLabel = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.red_compassLabel.sizePolicy().hasHeightForWidth())
        self.red_compassLabel.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.red_compassLabel.setFont(font)
        self.red_compassLabel.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.red_compassLabel.setFrameShadow(QtWidgets.QFrame.Plain)
        self.red_compassLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.red_compassLabel.setWordWrap(False)
        self.red_compassLabel.setObjectName("red_compassLabel")
        self.red_verticalLayout.addWidget(self.red_compassLabel)
        self.red_compassPlotWidget = QtWidgets.QGraphicsView(self.layoutWidget)
        self.red_compassPlotWidget.setObjectName("red_compassPlotWidget")
        self.red_verticalLayout.addWidget(self.red_compassPlotWidget)
        self.red_gpsLabel = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.red_gpsLabel.sizePolicy().hasHeightForWidth())
        self.red_gpsLabel.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.red_gpsLabel.setFont(font)
        self.red_gpsLabel.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.red_gpsLabel.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.red_gpsLabel.setFrameShadow(QtWidgets.QFrame.Plain)
        self.red_gpsLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.red_gpsLabel.setObjectName("red_gpsLabel")
        self.red_verticalLayout.addWidget(self.red_gpsLabel)
        self.red_gpsdisplayLabel = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.red_gpsdisplayLabel.sizePolicy().hasHeightForWidth())
        self.red_gpsdisplayLabel.setSizePolicy(sizePolicy)
        self.red_gpsdisplayLabel.setObjectName("red_gpsdisplayLabel")
        self.red_verticalLayout.addWidget(self.red_gpsdisplayLabel)
        self.red_timeLabel = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.red_timeLabel.sizePolicy().hasHeightForWidth())
        self.red_timeLabel.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setBold(True)
        font.setUnderline(False)
        font.setWeight(75)
        font.setKerning(True)
        self.red_timeLabel.setFont(font)
        self.red_timeLabel.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.red_timeLabel.setFrameShadow(QtWidgets.QFrame.Plain)
        self.red_timeLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.red_timeLabel.setObjectName("red_timeLabel")
        self.red_verticalLayout.addWidget(self.red_timeLabel)
        self.red_timeProgressBar = QtWidgets.QProgressBar(self.layoutWidget)
        self.red_timeProgressBar.setProperty("value", 24)
        self.red_timeProgressBar.setObjectName("red_timeProgressBar")
        self.red_verticalLayout.addWidget(self.red_timeProgressBar)
        self.horizontalLayout.addLayout(self.red_verticalLayout)
        self.blue_verticalLayout = QtWidgets.QVBoxLayout()
        self.blue_verticalLayout.setContentsMargins(-1, -1, -1, 0)
        self.blue_verticalLayout.setObjectName("blue_verticalLayout")
        self.blue_exportButton = QtWidgets.QPushButton(self.layoutWidget)
        self.blue_exportButton.setObjectName("blue_exportButton")
        self.blue_verticalLayout.addWidget(self.blue_exportButton)
        self.blue_realtimeButton = QtWidgets.QPushButton(self.layoutWidget)
        self.blue_realtimeButton.setObjectName("blue_realtimeButton")
        self.blue_verticalLayout.addWidget(self.blue_realtimeButton)
        self.blue_compassLabel = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.blue_compassLabel.sizePolicy().hasHeightForWidth())
        self.blue_compassLabel.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.blue_compassLabel.setFont(font)
        self.blue_compassLabel.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.blue_compassLabel.setFrameShadow(QtWidgets.QFrame.Plain)
        self.blue_compassLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.blue_compassLabel.setWordWrap(False)
        self.blue_compassLabel.setObjectName("blue_compassLabel")
        self.blue_verticalLayout.addWidget(self.blue_compassLabel)
        self.blue_compassPlotWidget = QtWidgets.QGraphicsView(self.layoutWidget)
        self.blue_compassPlotWidget.setObjectName("blue_compassPlotWidget")
        self.blue_verticalLayout.addWidget(self.blue_compassPlotWidget)
        self.blue_gpsLabel = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.blue_gpsLabel.sizePolicy().hasHeightForWidth())
        self.blue_gpsLabel.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.blue_gpsLabel.setFont(font)
        self.blue_gpsLabel.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.blue_gpsLabel.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.blue_gpsLabel.setFrameShadow(QtWidgets.QFrame.Plain)
        self.blue_gpsLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.blue_gpsLabel.setObjectName("blue_gpsLabel")
        self.blue_verticalLayout.addWidget(self.blue_gpsLabel)
        self.blue_gpsdisplayLabel = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.blue_gpsdisplayLabel.sizePolicy().hasHeightForWidth())
        self.blue_gpsdisplayLabel.setSizePolicy(sizePolicy)
        self.blue_gpsdisplayLabel.setObjectName("blue_gpsdisplayLabel")
        self.blue_verticalLayout.addWidget(self.blue_gpsdisplayLabel)
        self.blue_timeLabel = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.blue_timeLabel.sizePolicy().hasHeightForWidth())
        self.blue_timeLabel.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.blue_timeLabel.setFont(font)
        self.blue_timeLabel.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.blue_timeLabel.setFrameShadow(QtWidgets.QFrame.Plain)
        self.blue_timeLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.blue_timeLabel.setObjectName("blue_timeLabel")
        self.blue_verticalLayout.addWidget(self.blue_timeLabel)
        self.blue_timeProgressBar = QtWidgets.QProgressBar(self.layoutWidget)
        self.blue_timeProgressBar.setProperty("value", 24)
        self.blue_timeProgressBar.setObjectName("blue_timeProgressBar")
        self.blue_verticalLayout.addWidget(self.blue_timeProgressBar)
        self.horizontalLayout.addLayout(self.blue_verticalLayout)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1020, 28))
        self.menubar.setObjectName("menubar")
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        self.menuEdit = QtWidgets.QMenu(self.menubar)
        self.menuEdit.setObjectName("menuEdit")
        self.menuView = QtWidgets.QMenu(self.menubar)
        self.menuView.setObjectName("menuView")
        self.menuPreferences = QtWidgets.QMenu(self.menuView)
        self.menuPreferences.setObjectName("menuPreferences")
        self.menuBackground = QtWidgets.QMenu(self.menuPreferences)
        self.menuBackground.setObjectName("menuBackground")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionNew = QtWidgets.QAction(MainWindow)
        self.actionNew.setObjectName("actionNew")
        self.actionOpen = QtWidgets.QAction(MainWindow)
        self.actionOpen.setObjectName("actionOpen")
        self.actionImport = QtWidgets.QAction(MainWindow)
        self.actionImport.setObjectName("actionImport")
        self.actionExport = QtWidgets.QAction(MainWindow)
        self.actionExport.setObjectName("actionExport")
        self.actionAdd_Waypoint = QtWidgets.QAction(MainWindow)
        self.actionAdd_Waypoint.setObjectName("actionAdd_Waypoint")
        self.actionRemove_Waypoint = QtWidgets.QAction(MainWindow)
        self.actionRemove_Waypoint.setObjectName("actionRemove_Waypoint")
        self.actionRemove_Path = QtWidgets.QAction(MainWindow)
        self.actionRemove_Path.setObjectName("actionRemove_Path")
        self.actionRed_path = QtWidgets.QAction(MainWindow)
        self.actionRed_path.setObjectName("actionRed_path")
        self.actionBlue_path = QtWidgets.QAction(MainWindow)
        self.actionBlue_path.setObjectName("actionBlue_path")
        self.actionwhite = QtWidgets.QAction(MainWindow)
        self.actionwhite.setObjectName("actionwhite")
        self.actionslategray = QtWidgets.QAction(MainWindow)
        self.actionslategray.setObjectName("actionslategray")
        self.actioncharcoal = QtWidgets.QAction(MainWindow)
        self.actioncharcoal.setObjectName("actioncharcoal")
        self.actionSmall = QtWidgets.QAction(MainWindow)
        self.actionSmall.setObjectName("actionSmall")
        self.actionMedium = QtWidgets.QAction(MainWindow)
        self.actionMedium.setObjectName("actionMedium")
        self.menuFile.addAction(self.actionNew)
        self.menuFile.addAction(self.actionOpen)
        self.menuFile.addSeparator()
        self.menuFile.addAction(self.actionImport)
        self.menuFile.addAction(self.actionExport)
        self.menuEdit.addAction(self.actionAdd_Waypoint)
        self.menuEdit.addAction(self.actionRemove_Waypoint)
        self.menuBackground.addAction(self.actionwhite)
        self.menuBackground.addAction(self.actionslategray)
        self.menuBackground.addAction(self.actioncharcoal)
        self.menuPreferences.addAction(self.menuBackground.menuAction())
        self.menuView.addAction(self.actionRed_path)
        self.menuView.addAction(self.actionBlue_path)
        self.menuView.addSeparator()
        self.menuView.addAction(self.menuPreferences.menuAction())
        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuEdit.menuAction())
        self.menubar.addAction(self.menuView.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.red_exportButton.setText(_translate("MainWindow", "Export Waypoints"))
        self.red_realtimeButton.setText(_translate("MainWindow", "Realtime view"))
        self.red_compassLabel.setText(_translate("MainWindow", "Compass"))
        self.red_gpsLabel.setText(_translate("MainWindow", "GPS"))
        self.red_gpsdisplayLabel.setText(_translate("MainWindow", "TextLabel"))
        self.red_timeLabel.setText(_translate("MainWindow", "Time"))
        self.blue_exportButton.setText(_translate("MainWindow", "Export Waypoints"))
        self.blue_realtimeButton.setText(_translate("MainWindow", "Realtime view"))
        self.blue_compassLabel.setText(_translate("MainWindow", "Compass"))
        self.blue_gpsLabel.setText(_translate("MainWindow", "GPS"))
        self.blue_gpsdisplayLabel.setText(_translate("MainWindow", "TextLabel"))
        self.blue_timeLabel.setText(_translate("MainWindow", "Time"))
        self.menuFile.setTitle(_translate("MainWindow", "File"))
        self.menuEdit.setTitle(_translate("MainWindow", "Edit"))
        self.menuView.setTitle(_translate("MainWindow", "View"))
        self.menuPreferences.setTitle(_translate("MainWindow", "Preferences"))
        self.menuBackground.setTitle(_translate("MainWindow", "Background"))
        self.actionNew.setText(_translate("MainWindow", "New..."))
        self.actionNew.setToolTip(_translate("MainWindow", "Create a New Path"))
        self.actionNew.setShortcut(_translate("MainWindow", "Ctrl+N"))
        self.actionOpen.setText(_translate("MainWindow", "Open File"))
        self.actionOpen.setShortcut(_translate("MainWindow", "Ctrl+O"))
        self.actionImport.setText(_translate("MainWindow", "Import..."))
        self.actionImport.setShortcut(_translate("MainWindow", "Ctrl+I"))
        self.actionExport.setText(_translate("MainWindow", "Export..."))
        self.actionExport.setShortcut(_translate("MainWindow", "Ctrl+E"))
        self.actionAdd_Waypoint.setText(_translate("MainWindow", "Add Waypoint"))
        self.actionAdd_Waypoint.setShortcut(_translate("MainWindow", "F1"))
        self.actionRemove_Waypoint.setText(_translate("MainWindow", "Remove Waypoint"))
        self.actionRemove_Waypoint.setShortcut(_translate("MainWindow", "F2"))
        self.actionRemove_Path.setText(_translate("MainWindow", "Remove Path"))
        self.actionRemove_Path.setShortcut(_translate("MainWindow", "F3"))
        self.actionRed_path.setText(_translate("MainWindow", "Red path"))
        self.actionRed_path.setShortcut(_translate("MainWindow", "R"))
        self.actionBlue_path.setText(_translate("MainWindow", "Blue path"))
        self.actionBlue_path.setShortcut(_translate("MainWindow", "B"))
        self.actionwhite.setText(_translate("MainWindow", "white"))
        self.actionslategray.setText(_translate("MainWindow", "slategray"))
        self.actioncharcoal.setText(_translate("MainWindow", "charcoal"))
        self.actionSmall.setText(_translate("MainWindow", "Small"))
        self.actionMedium.setText(_translate("MainWindow", "Medium"))
