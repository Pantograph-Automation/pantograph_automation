# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'ManualWindow.ui'
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
from PySide6.QtWidgets import (QApplication, QFrame, QHBoxLayout, QLabel,
    QPushButton, QSizePolicy, QSlider, QSpacerItem,
    QVBoxLayout, QWidget)

class Ui_ManualWindow(object):
    def setupUi(self, ManualWindow):
        if not ManualWindow.objectName():
            ManualWindow.setObjectName(u"ManualWindow")
        ManualWindow.resize(600, 400)
        ManualWindow.setMinimumSize(QSize(600, 400))
        ManualWindow.setMaximumSize(QSize(600, 400))
        ManualWindow.setStyleSheet(u"/* --- Global & Base Widget --- */\n"
"QWidget {\n"
"    background-color: #ffffff;\n"
"    color: #333333;\n"
"    font-family: \"Segoe UI\", \"Helvetica Neue\", sans-serif;\n"
"    font-size: 10px;\n"
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
"    border-radius: 15px;\n"
"    padding: 5px 5px;\n"
"    font-weight: 800;\n"
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
"/* Secondary Button (Set objectName to 'secondaryButton') - Outlined "
                        "*/\n"
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
        self.frame = QFrame(ManualWindow)
        self.frame.setObjectName(u"frame")
        self.frame.setGeometry(QRect(210, 40, 300, 300))
        self.frame.setMinimumSize(QSize(300, 300))
        self.frame.setMaximumSize(QSize(300, 300))
        self.frame.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout = QHBoxLayout(self.frame)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.verticalLayout_4 = QVBoxLayout()
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.verticalSpacer = QSpacerItem(30, 30, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.verticalLayout_4.addItem(self.verticalSpacer)

        self.posXButton = QPushButton(self.frame)
        self.posXButton.setObjectName(u"posXButton")
        self.posXButton.setMinimumSize(QSize(60, 60))
        self.posXButton.setMaximumSize(QSize(60, 60))

        self.verticalLayout_4.addWidget(self.posXButton)

        self.verticalSpacer_2 = QSpacerItem(30, 30, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.verticalLayout_4.addItem(self.verticalSpacer_2)


        self.horizontalLayout.addLayout(self.verticalLayout_4)

        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.negYButton = QPushButton(self.frame)
        self.negYButton.setObjectName(u"negYButton")
        self.negYButton.setMinimumSize(QSize(60, 60))
        self.negYButton.setMaximumSize(QSize(60, 60))

        self.verticalLayout_2.addWidget(self.negYButton)

        self.posYButton = QPushButton(self.frame)
        self.posYButton.setObjectName(u"posYButton")
        self.posYButton.setMinimumSize(QSize(60, 60))
        self.posYButton.setMaximumSize(QSize(60, 60))

        self.verticalLayout_2.addWidget(self.posYButton)


        self.horizontalLayout.addLayout(self.verticalLayout_2)

        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalSpacer_4 = QSpacerItem(30, 30, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.verticalLayout_3.addItem(self.verticalSpacer_4)

        self.negXButton = QPushButton(self.frame)
        self.negXButton.setObjectName(u"negXButton")
        self.negXButton.setMinimumSize(QSize(60, 60))
        self.negXButton.setMaximumSize(QSize(60, 60))

        self.verticalLayout_3.addWidget(self.negXButton)

        self.verticalSpacer_3 = QSpacerItem(30, 30, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.verticalLayout_3.addItem(self.verticalSpacer_3)


        self.horizontalLayout.addLayout(self.verticalLayout_3)

        self.xLabel = QLabel(ManualWindow)
        self.xLabel.setObjectName(u"xLabel")
        self.xLabel.setGeometry(QRect(170, 170, 51, 31))
        self.xLabel.setStyleSheet(u"QLabel {\n"
"	background-color: white;\n"
"    font-weight: bold;\n"
"	color: black;\n"
"}")
        self.yLabel = QLabel(ManualWindow)
        self.yLabel.setObjectName(u"yLabel")
        self.yLabel.setGeometry(QRect(530, 60, 51, 31))
        self.yLabel.setStyleSheet(u"QLabel {\n"
"	background-color: white;\n"
"    font-weight: bold;\n"
"	color: black;\n"
"}")
        self.yLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.zLabel = QLabel(ManualWindow)
        self.zLabel.setObjectName(u"zLabel")
        self.zLabel.setGeometry(QRect(340, 10, 51, 31))
        self.zLabel.setStyleSheet(u"QLabel {\n"
"	background-color: white;\n"
"    font-weight: bold;\n"
"	color: black;\n"
"}")
        self.headerLabel = QLabel(ManualWindow)
        self.headerLabel.setObjectName(u"headerLabel")
        self.headerLabel.setGeometry(QRect(10, 10, 270, 30))
        self.headerLabel.setMinimumSize(QSize(270, 30))
        self.layoutWidget = QWidget(ManualWindow)
        self.layoutWidget.setObjectName(u"layoutWidget")
        self.layoutWidget.setGeometry(QRect(30, 140, 89, 72))
        self.verticalLayout_5 = QVBoxLayout(self.layoutWidget)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.label_8 = QLabel(self.layoutWidget)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setMinimumSize(QSize(30, 30))
        self.label_8.setMaximumSize(QSize(30, 30))
        self.label_8.setStyleSheet(u"QLabel {\n"
"    background-color: white;\n"
"    border-radius: 15px;\n"
"    border: 2px solid black;\n"
"	font-weight: bold;\n"
"	color: black;\n"
"}")
        self.label_8.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.horizontalLayout_3.addWidget(self.label_8)

        self.j1Label = QLabel(self.layoutWidget)
        self.j1Label.setObjectName(u"j1Label")
        self.j1Label.setStyleSheet(u"QLabel {\n"
"	background-color: white;\n"
"    font-weight: bold;\n"
"	color: black;\n"
"}")

        self.horizontalLayout_3.addWidget(self.j1Label)


        self.verticalLayout_5.addLayout(self.horizontalLayout_3)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.label_9 = QLabel(self.layoutWidget)
        self.label_9.setObjectName(u"label_9")
        self.label_9.setMinimumSize(QSize(30, 30))
        self.label_9.setMaximumSize(QSize(30, 30))
        self.label_9.setStyleSheet(u"QLabel {\n"
"    background-color: white;\n"
"    border-radius: 15px;\n"
"    border: 2px solid black;\n"
"	font-weight: bold;\n"
"	color: black;\n"
"}")
        self.label_9.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.horizontalLayout_4.addWidget(self.label_9)

        self.j2Label = QLabel(self.layoutWidget)
        self.j2Label.setObjectName(u"j2Label")
        self.j2Label.setStyleSheet(u"QLabel {\n"
"	background-color: white;\n"
"    font-weight: bold;\n"
"	color: black;\n"
"}")

        self.horizontalLayout_4.addWidget(self.j2Label)


        self.verticalLayout_5.addLayout(self.horizontalLayout_4)

        self.layoutWidget1 = QWidget(ManualWindow)
        self.layoutWidget1.setObjectName(u"layoutWidget1")
        self.layoutWidget1.setGeometry(QRect(160, 350, 291, 24))
        self.horizontalLayout_5 = QHBoxLayout(self.layoutWidget1)
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.horizontalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.horizontalSlider = QSlider(self.layoutWidget1)
        self.horizontalSlider.setObjectName(u"horizontalSlider")
        self.horizontalSlider.setMaximum(999)
        self.horizontalSlider.setOrientation(Qt.Orientation.Horizontal)

        self.horizontalLayout_5.addWidget(self.horizontalSlider)

        self.yLabel_2 = QLabel(self.layoutWidget1)
        self.yLabel_2.setObjectName(u"yLabel_2")
        self.yLabel_2.setStyleSheet(u"QLabel {\n"
"	background-color: white;\n"
"    font-weight: bold;\n"
"	color: black;\n"
"}")

        self.horizontalLayout_5.addWidget(self.yLabel_2)

        self.widget = QWidget(ManualWindow)
        self.widget.setObjectName(u"widget")
        self.widget.setGeometry(QRect(520, 90, 61, 211))
        self.verticalLayout = QVBoxLayout(self.widget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.posZButton = QPushButton(self.widget)
        self.posZButton.setObjectName(u"posZButton")
        self.posZButton.setMinimumSize(QSize(60, 60))
        self.posZButton.setMaximumSize(QSize(60, 60))

        self.verticalLayout.addWidget(self.posZButton)

        self.negZButton = QPushButton(self.widget)
        self.negZButton.setObjectName(u"negZButton")
        self.negZButton.setMinimumSize(QSize(60, 60))
        self.negZButton.setMaximumSize(QSize(60, 60))

        self.verticalLayout.addWidget(self.negZButton)


        self.retranslateUi(ManualWindow)

        QMetaObject.connectSlotsByName(ManualWindow)
    # setupUi

    def retranslateUi(self, ManualWindow):
        ManualWindow.setWindowTitle(QCoreApplication.translate("ManualWindow", u"Form", None))
        self.posXButton.setText(QCoreApplication.translate("ManualWindow", u"+x", None))
        self.negYButton.setText(QCoreApplication.translate("ManualWindow", u"-y", None))
        self.posYButton.setText(QCoreApplication.translate("ManualWindow", u"+y", None))
        self.negXButton.setText(QCoreApplication.translate("ManualWindow", u"-x", None))
        self.xLabel.setText(QCoreApplication.translate("ManualWindow", u"0.0 m", None))
        self.yLabel.setText(QCoreApplication.translate("ManualWindow", u"0.0 m", None))
        self.zLabel.setText(QCoreApplication.translate("ManualWindow", u"0.0 m", None))
        self.headerLabel.setText(QCoreApplication.translate("ManualWindow", u"Manual Control", None))
        self.label_8.setText(QCoreApplication.translate("ManualWindow", u"J1", None))
        self.j1Label.setText(QCoreApplication.translate("ManualWindow", u"0.0 deg", None))
        self.label_9.setText(QCoreApplication.translate("ManualWindow", u"J2", None))
        self.j2Label.setText(QCoreApplication.translate("ManualWindow", u"0.0 deg", None))
        self.yLabel_2.setText(QCoreApplication.translate("ManualWindow", u"0.01 m", None))
        self.posZButton.setText(QCoreApplication.translate("ManualWindow", u"+z", None))
        self.negZButton.setText(QCoreApplication.translate("ManualWindow", u"-z", None))
    # retranslateUi

