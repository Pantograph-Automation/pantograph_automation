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
        MainWindow.resize(680, 499)
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
        self.label_11 = QLabel(self.frame)
        self.label_11.setObjectName(u"label_11")
        self.label_11.setMinimumSize(QSize(100, 40))
        self.label_11.setStyleSheet(u"QLabel {\n"
"    background-color: black;\n"
"    border-radius: 20px;  /* makes a circle if width = height */\n"
"}")

        self.verticalLayout.addWidget(self.label_11)

        self.statusLabel2 = QLabel(self.frame)
        self.statusLabel2.setObjectName(u"statusLabel2")
        self.statusLabel2.setMinimumSize(QSize(100, 40))
        self.statusLabel2.setStyleSheet(u"QLabel {\n"
"    background-color: black;\n"
"    border-radius: 20px;  /* makes a circle if width = height */\n"
"}")

        self.verticalLayout.addWidget(self.statusLabel2)


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
        self.label = QLabel(self.centralwidget)
        self.label.setObjectName(u"label")
        self.label.setMinimumSize(QSize(150, 150))
        self.label.setMaximumSize(QSize(150, 150))
        self.label.setStyleSheet(u"QLabel {\n"
"    background-color: black;\n"
"    border-radius: 75px;  /* makes a circle if width = height */\n"
"}")

        self.horizontalLayout.addWidget(self.label)

        self.label_2 = QLabel(self.centralwidget)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setMinimumSize(QSize(150, 150))
        self.label_2.setMaximumSize(QSize(150, 150))
        self.label_2.setStyleSheet(u"QLabel {\n"
"    background-color: black;\n"
"    border-radius: 75px;  /* makes a circle if width = height */\n"
"}")

        self.horizontalLayout.addWidget(self.label_2)

        self.label_3 = QLabel(self.centralwidget)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setMinimumSize(QSize(150, 150))
        self.label_3.setMaximumSize(QSize(150, 150))
        self.label_3.setStyleSheet(u"QLabel {\n"
"    background-color: black;\n"
"    border-radius: 75px;  /* makes a circle if width = height */\n"
"}")

        self.horizontalLayout.addWidget(self.label_3)

        self.label_4 = QLabel(self.centralwidget)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setMinimumSize(QSize(150, 150))
        self.label_4.setMaximumSize(QSize(150, 150))
        self.label_4.setStyleSheet(u"QLabel {\n"
"    background-color: black;\n"
"    border-radius: 75px;  /* makes a circle if width = height */\n"
"}")

        self.horizontalLayout.addWidget(self.label_4)


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
        self.lineEdit = QLineEdit(self.frame_2)
        self.lineEdit.setObjectName(u"lineEdit")
        self.lineEdit.setMinimumSize(QSize(40, 25))

        self.verticalLayout_2.addWidget(self.lineEdit)

        self.comboBox = QComboBox(self.frame_2)
        self.comboBox.setObjectName(u"comboBox")
        self.comboBox.setMinimumSize(QSize(40, 25))

        self.verticalLayout_2.addWidget(self.comboBox)


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
        self.lineEdit_2 = QLineEdit(self.frame_3)
        self.lineEdit_2.setObjectName(u"lineEdit_2")
        self.lineEdit_2.setMinimumSize(QSize(40, 25))

        self.verticalLayout_3.addWidget(self.lineEdit_2)

        self.comboBox_2 = QComboBox(self.frame_3)
        self.comboBox_2.setObjectName(u"comboBox_2")
        self.comboBox_2.setMinimumSize(QSize(40, 25))

        self.verticalLayout_3.addWidget(self.comboBox_2)


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
        self.lineEdit_3 = QLineEdit(self.frame_4)
        self.lineEdit_3.setObjectName(u"lineEdit_3")
        self.lineEdit_3.setMinimumSize(QSize(40, 25))

        self.verticalLayout_4.addWidget(self.lineEdit_3)

        self.comboBox_3 = QComboBox(self.frame_4)
        self.comboBox_3.setObjectName(u"comboBox_3")
        self.comboBox_3.setMinimumSize(QSize(40, 25))

        self.verticalLayout_4.addWidget(self.comboBox_3)


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
        self.lineEdit_4 = QLineEdit(self.frame_5)
        self.lineEdit_4.setObjectName(u"lineEdit_4")
        self.lineEdit_4.setMinimumSize(QSize(40, 25))

        self.verticalLayout_5.addWidget(self.lineEdit_4)

        self.comboBox_4 = QComboBox(self.frame_5)
        self.comboBox_4.setObjectName(u"comboBox_4")
        self.comboBox_4.setMinimumSize(QSize(40, 25))

        self.verticalLayout_5.addWidget(self.comboBox_4)


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
        self.pushButton_6 = QPushButton(self.centralwidget)
        self.pushButton_6.setObjectName(u"pushButton_6")
        self.pushButton_6.setMinimumSize(QSize(100, 40))
        self.pushButton_6.setMaximumSize(QSize(100, 40))

        self.horizontalLayout_2.addWidget(self.pushButton_6)

        self.pushButton_5 = QPushButton(self.centralwidget)
        self.pushButton_5.setObjectName(u"pushButton_5")
        self.pushButton_5.setMinimumSize(QSize(100, 40))
        self.pushButton_5.setMaximumSize(QSize(100, 40))

        self.horizontalLayout_2.addWidget(self.pushButton_5)

        self.secondaryButton = QPushButton(self.centralwidget)
        self.secondaryButton.setObjectName(u"secondaryButton")
        self.secondaryButton.setMinimumSize(QSize(100, 40))
        self.secondaryButton.setMaximumSize(QSize(100, 40))

        self.horizontalLayout_2.addWidget(self.secondaryButton)


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
        self.label_11.setText(QCoreApplication.translate("MainWindow", u"Calibrated", None))
        self.statusLabel2.setText(QCoreApplication.translate("MainWindow", u"Running", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"Dish A", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"Dish B", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"Dish C", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"Dish D", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"Dish A", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"Dish B", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"Dish C", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"Dish D", None))
        self.pushButton_6.setText(QCoreApplication.translate("MainWindow", u"Validate", None))
        self.pushButton_5.setText(QCoreApplication.translate("MainWindow", u"Calibrate", None))
        self.secondaryButton.setText(QCoreApplication.translate("MainWindow", u"Run", None))
    # retranslateUi

