# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'MainWindow.ui'
##
## Created by: Qt User Interface Compiler version 6.10.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QComboBox, QFrame, QHBoxLayout,
    QLabel, QLayout, QLineEdit, QMainWindow,
    QPushButton, QSizePolicy, QSpacerItem, QStatusBar,
    QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(680, 443)
        MainWindow.setStyleSheet(u"/* --- Global & Base Widget --- */\n"
"QWidget {\n"
"    background-color: #ffffff;\n"
"    color: #333333;\n"
"    font-family: \"Segoe UI\", \"Helvetica Neue\", sans-serif;\n"
"    font-size: 14px;\n"
"}\n"
"\n"
"/* --- Labels --- */\n"
"/* Normal Text Label */\n"
"QLabel {\n"
"    color: #666666;\n"
"    background: transparent;\n"
"}\n"
"\n"
"/* Header Label (Set objectName to 'headerLabel') */\n"
"QLabel#headerLabel {\n"
"    font-size: 24px;\n"
"    font-weight: bold;\n"
"    color: #1a1a1a;\n"
"    margin-bottom: 10px;\n"
"}\n"
"\n"
"/* --- Buttons --- */\n"
"/* Primary Button (Default) - Dark & Solid */\n"
"QPushButton {\n"
"    background-color: #1a1a1a;\n"
"    color: #ffffff;\n"
"    border: none;\n"
"    border-radius: 5px;\n"
"    padding: 10px 20px;\n"
"    font-weight: 600;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #333333;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #000000;\n"
"}\n"
"\n"
"/* Secondary Button (Set objectName to 'secondaryButton') - Outlined"
                        " */\n"
"QPushButton#secondaryButton {\n"
"    background-color: transparent;\n"
"    color: #1a1a1a;\n"
"    border: 1px solid #1a1a1a;\n"
"}\n"
"\n"
"QPushButton#secondaryButton:hover {\n"
"    background-color: #f4f4f4;\n"
"}\n"
"\n"
"QPushButton#secondaryButton:pressed {\n"
"    background-color: #e0e0e0;\n"
"}\n"
"\n"
"/* --- Checkbox --- */\n"
"QCheckBox {\n"
"    spacing: 10px;\n"
"}\n"
"\n"
"QCheckBox::indicator {\n"
"    width: 18px;\n"
"    height: 18px;\n"
"    border: 1px solid #cccccc;\n"
"    border-radius: 3px;\n"
"    background-color: white;\n"
"}\n"
"\n"
"QCheckBox::indicator:hover {\n"
"    border: 1px solid #1a1a1a;\n"
"}\n"
"\n"
"QCheckBox::indicator:checked {\n"
"    background-color: #1a1a1a;\n"
"    border: 1px solid #1a1a1a;\n"
"    /* Note: To show a checkmark icon, you would typically \n"
"       reference a resource file image here */\n"
"}")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.verticalLayout_6 = QVBoxLayout(self.centralwidget)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.horizontalSpacer_2 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_5.addItem(self.horizontalSpacer_2)

        self.frame = QFrame(self.centralwidget)
        self.frame.setObjectName(u"frame")
        self.frame.setMinimumSize(QSize(300, 120))
        self.frame.setStyleSheet(u"")
        self.frame.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_6 = QHBoxLayout(self.frame)
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.headerLabel = QLabel(self.frame)
        self.headerLabel.setObjectName(u"headerLabel")
        self.headerLabel.setMinimumSize(QSize(130, 90))

        self.horizontalLayout_6.addWidget(self.headerLabel)

        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(-1, 5, -1, 5)
        self.topStatusLabel = QLabel(self.frame)
        self.topStatusLabel.setObjectName(u"topStatusLabel")
        self.topStatusLabel.setMinimumSize(QSize(100, 40))
        self.topStatusLabel.setStyleSheet(u"QLabel {\n"
"    background-color: black;\n"
"    border-radius: 20px;  /* makes a circle if width = height */\n"
"}")
        self.topStatusLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout.addWidget(self.topStatusLabel)

        self.bottomStatusLabel = QLabel(self.frame)
        self.bottomStatusLabel.setObjectName(u"bottomStatusLabel")
        self.bottomStatusLabel.setMinimumSize(QSize(100, 40))
        self.bottomStatusLabel.setStyleSheet(u"QLabel {\n"
"    background-color: black;\n"
"    border-radius: 20px;  /* makes a circle if width = height */\n"
"}")
        self.bottomStatusLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout.addWidget(self.bottomStatusLabel)


        self.horizontalLayout_6.addLayout(self.verticalLayout)


        self.horizontalLayout_5.addWidget(self.frame)

        self.horizontalSpacer_3 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_5.addItem(self.horizontalSpacer_3)


        self.verticalLayout_6.addLayout(self.horizontalLayout_5)

        self.verticalSpacer_2 = QSpacerItem(17, 13, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Minimum)

        self.verticalLayout_6.addItem(self.verticalSpacer_2)

        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setSpacing(20)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setSizeConstraint(QLayout.SizeConstraint.SetDefaultConstraint)
        self.dishLabelA = QLabel(self.centralwidget)
        self.dishLabelA.setObjectName(u"dishLabelA")
        self.dishLabelA.setMinimumSize(QSize(100, 100))
        self.dishLabelA.setMaximumSize(QSize(100, 100))
        self.dishLabelA.setStyleSheet(u"QLabel {\n"
"    background-color: black;\n"
"    border-radius: 50px;  /* makes a circle if width = height */\n"
"}")
        self.dishLabelA.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.horizontalLayout.addWidget(self.dishLabelA)

        self.dishLabelB = QLabel(self.centralwidget)
        self.dishLabelB.setObjectName(u"dishLabelB")
        self.dishLabelB.setMinimumSize(QSize(100, 100))
        self.dishLabelB.setMaximumSize(QSize(100, 100))
        self.dishLabelB.setStyleSheet(u"QLabel {\n"
"    background-color: black;\n"
"    border-radius: 50px;  /* makes a circle if width = height */\n"
"}")
        self.dishLabelB.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.horizontalLayout.addWidget(self.dishLabelB)

        self.dishLabelC = QLabel(self.centralwidget)
        self.dishLabelC.setObjectName(u"dishLabelC")
        self.dishLabelC.setMinimumSize(QSize(100, 100))
        self.dishLabelC.setMaximumSize(QSize(100, 100))
        self.dishLabelC.setStyleSheet(u"QLabel {\n"
"    background-color: black;\n"
"    border-radius: 50px;  /* makes a circle if width = height */\n"
"}")
        self.dishLabelC.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.horizontalLayout.addWidget(self.dishLabelC)

        self.dishLabelD = QLabel(self.centralwidget)
        self.dishLabelD.setObjectName(u"dishLabelD")
        self.dishLabelD.setMinimumSize(QSize(100, 100))
        self.dishLabelD.setMaximumSize(QSize(100, 100))
        self.dishLabelD.setStyleSheet(u"QLabel {\n"
"    background-color: black;\n"
"    border-radius: 50px;  /* makes a circle if width = height */\n"
"}\n"
"")
        self.dishLabelD.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.horizontalLayout.addWidget(self.dishLabelD)


        self.verticalLayout_6.addLayout(self.horizontalLayout)

        self.horizontalLayout_11 = QHBoxLayout()
        self.horizontalLayout_11.setObjectName(u"horizontalLayout_11")
        self.frame_2 = QFrame(self.centralwidget)
        self.frame_2.setObjectName(u"frame_2")
        self.frame_2.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame_2.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_7 = QHBoxLayout(self.frame_2)
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.label_5 = QLabel(self.frame_2)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setMinimumSize(QSize(60, 30))
        self.label_5.setStyleSheet(u"QLabel {\n"
"    font-size: 18px;\n"
"    font-weight: bold;\n"
"    color: #1a1a1a;\n"
"    margin-bottom: 10px;\n"
"}\n"
"")
        self.label_5.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.horizontalLayout_7.addWidget(self.label_5)

        self.horizontalSpacer = QSpacerItem(0, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_7.addItem(self.horizontalSpacer)

        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.lineEditA = QLineEdit(self.frame_2)
        self.lineEditA.setObjectName(u"lineEditA")
        self.lineEditA.setMinimumSize(QSize(40, 25))

        self.verticalLayout_2.addWidget(self.lineEditA)

        self.comboBoxA = QComboBox(self.frame_2)
        self.comboBoxA.setObjectName(u"comboBoxA")
        self.comboBoxA.setMinimumSize(QSize(40, 25))
        self.comboBoxA.setMaxVisibleItems(5)
        self.comboBoxA.setMaxCount(5)

        self.verticalLayout_2.addWidget(self.comboBoxA)


        self.horizontalLayout_7.addLayout(self.verticalLayout_2)


        self.horizontalLayout_11.addWidget(self.frame_2)

        self.frame_3 = QFrame(self.centralwidget)
        self.frame_3.setObjectName(u"frame_3")
        self.frame_3.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame_3.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_8 = QHBoxLayout(self.frame_3)
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.label_6 = QLabel(self.frame_3)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setMinimumSize(QSize(60, 30))
        self.label_6.setStyleSheet(u"QLabel {\n"
"    font-size: 18px;\n"
"    font-weight: bold;\n"
"    color: #1a1a1a;\n"
"    margin-bottom: 10px;\n"
"}\n"
"")
        self.label_6.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.horizontalLayout_8.addWidget(self.label_6)

        self.horizontalSpacer_4 = QSpacerItem(1, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_8.addItem(self.horizontalSpacer_4)

        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.lineEditB = QLineEdit(self.frame_3)
        self.lineEditB.setObjectName(u"lineEditB")
        self.lineEditB.setMinimumSize(QSize(40, 25))

        self.verticalLayout_3.addWidget(self.lineEditB)

        self.comboBoxB = QComboBox(self.frame_3)
        self.comboBoxB.setObjectName(u"comboBoxB")
        self.comboBoxB.setMinimumSize(QSize(40, 25))

        self.verticalLayout_3.addWidget(self.comboBoxB)


        self.horizontalLayout_8.addLayout(self.verticalLayout_3)


        self.horizontalLayout_11.addWidget(self.frame_3)

        self.frame_4 = QFrame(self.centralwidget)
        self.frame_4.setObjectName(u"frame_4")
        self.frame_4.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame_4.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_9 = QHBoxLayout(self.frame_4)
        self.horizontalLayout_9.setObjectName(u"horizontalLayout_9")
        self.label_7 = QLabel(self.frame_4)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setMinimumSize(QSize(60, 30))
        self.label_7.setStyleSheet(u"QLabel {\n"
"    font-size: 18px;\n"
"    font-weight: bold;\n"
"    color: #1a1a1a;\n"
"    margin-bottom: 10px;\n"
"}\n"
"")
        self.label_7.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.horizontalLayout_9.addWidget(self.label_7)

        self.horizontalSpacer_7 = QSpacerItem(1, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_9.addItem(self.horizontalSpacer_7)

        self.verticalLayout_4 = QVBoxLayout()
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.lineEditC = QLineEdit(self.frame_4)
        self.lineEditC.setObjectName(u"lineEditC")
        self.lineEditC.setMinimumSize(QSize(40, 25))

        self.verticalLayout_4.addWidget(self.lineEditC)

        self.comboBoxC = QComboBox(self.frame_4)
        self.comboBoxC.setObjectName(u"comboBoxC")
        self.comboBoxC.setMinimumSize(QSize(40, 25))

        self.verticalLayout_4.addWidget(self.comboBoxC)


        self.horizontalLayout_9.addLayout(self.verticalLayout_4)


        self.horizontalLayout_11.addWidget(self.frame_4)

        self.frame_5 = QFrame(self.centralwidget)
        self.frame_5.setObjectName(u"frame_5")
        self.frame_5.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame_5.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_10 = QHBoxLayout(self.frame_5)
        self.horizontalLayout_10.setObjectName(u"horizontalLayout_10")
        self.label_8 = QLabel(self.frame_5)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setMinimumSize(QSize(60, 30))
        self.label_8.setStyleSheet(u"QLabel {\n"
"    font-size: 18px;\n"
"    font-weight: bold;\n"
"    color: #1a1a1a;\n"
"    margin-bottom: 10px;\n"
"}\n"
"")
        self.label_8.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.horizontalLayout_10.addWidget(self.label_8)

        self.horizontalSpacer_8 = QSpacerItem(1, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_10.addItem(self.horizontalSpacer_8)

        self.verticalLayout_5 = QVBoxLayout()
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.lineEditD = QLineEdit(self.frame_5)
        self.lineEditD.setObjectName(u"lineEditD")
        self.lineEditD.setMinimumSize(QSize(40, 25))

        self.verticalLayout_5.addWidget(self.lineEditD)

        self.comboBoxD = QComboBox(self.frame_5)
        self.comboBoxD.setObjectName(u"comboBoxD")
        self.comboBoxD.setMinimumSize(QSize(40, 25))

        self.verticalLayout_5.addWidget(self.comboBoxD)


        self.horizontalLayout_10.addLayout(self.verticalLayout_5)


        self.horizontalLayout_11.addWidget(self.frame_5)


        self.verticalLayout_6.addLayout(self.horizontalLayout_11)

        self.verticalSpacer = QSpacerItem(17, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Minimum)

        self.verticalLayout_6.addItem(self.verticalSpacer)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.horizontalSpacer_6 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_4.addItem(self.horizontalSpacer_6)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.manualButton = QPushButton(self.centralwidget)
        self.manualButton.setObjectName(u"manualButton")
        self.manualButton.setMinimumSize(QSize(100, 40))
        self.manualButton.setMaximumSize(QSize(100, 40))

        self.horizontalLayout_2.addWidget(self.manualButton)

        self.calibrateButton = QPushButton(self.centralwidget)
        self.calibrateButton.setObjectName(u"calibrateButton")
        self.calibrateButton.setMinimumSize(QSize(100, 40))
        self.calibrateButton.setMaximumSize(QSize(100, 40))

        self.horizontalLayout_2.addWidget(self.calibrateButton)

        self.runButton = QPushButton(self.centralwidget)
        self.runButton.setObjectName(u"runButton")
        self.runButton.setMinimumSize(QSize(100, 40))
        self.runButton.setMaximumSize(QSize(100, 40))

        self.horizontalLayout_2.addWidget(self.runButton)


        self.horizontalLayout_4.addLayout(self.horizontalLayout_2)

        self.horizontalSpacer_5 = QSpacerItem(20, 20, QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_4.addItem(self.horizontalSpacer_5)


        self.verticalLayout_6.addLayout(self.horizontalLayout_4)

        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.headerLabel.setText(QCoreApplication.translate("MainWindow", u"STATUS", None))
        self.topStatusLabel.setText(QCoreApplication.translate("MainWindow", u"Calibrated", None))
        self.bottomStatusLabel.setText(QCoreApplication.translate("MainWindow", u"Running", None))
        self.dishLabelA.setText(QCoreApplication.translate("MainWindow", u"Dish A", None))
        self.dishLabelB.setText(QCoreApplication.translate("MainWindow", u"Dish B", None))
        self.dishLabelC.setText(QCoreApplication.translate("MainWindow", u"Dish C", None))
        self.dishLabelD.setText(QCoreApplication.translate("MainWindow", u"Dish D", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"Dish A", None))
        self.lineEditA.setPlaceholderText(QCoreApplication.translate("MainWindow", u"25", None))
        self.comboBoxA.setPlaceholderText(QCoreApplication.translate("MainWindow", u"Type", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"Dish B", None))
        self.lineEditB.setPlaceholderText(QCoreApplication.translate("MainWindow", u"25", None))
        self.comboBoxB.setPlaceholderText(QCoreApplication.translate("MainWindow", u"Type", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"Dish C", None))
        self.lineEditC.setPlaceholderText(QCoreApplication.translate("MainWindow", u"25", None))
        self.comboBoxC.setPlaceholderText(QCoreApplication.translate("MainWindow", u"Type", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"Dish D", None))
        self.lineEditD.setPlaceholderText(QCoreApplication.translate("MainWindow", u"25", None))
        self.comboBoxD.setPlaceholderText(QCoreApplication.translate("MainWindow", u"Type", None))
        self.manualButton.setText(QCoreApplication.translate("MainWindow", u"Manual", None))
        self.calibrateButton.setText(QCoreApplication.translate("MainWindow", u"Calibrate", None))
        self.runButton.setText(QCoreApplication.translate("MainWindow", u"Run", None))
    # retranslateUi

