# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'CalibrateWindow.ui'
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
from PySide6.QtWidgets import (QApplication, QCheckBox, QHBoxLayout, QLabel,
    QPushButton, QSizePolicy, QSpacerItem, QVBoxLayout,
    QWidget)

class Ui_CalibrateWindow(object):
    def setupUi(self, CalibrateWindow):
        if not CalibrateWindow.objectName():
            CalibrateWindow.setObjectName(u"CalibrateWindow")
        CalibrateWindow.resize(380, 364)
        CalibrateWindow.setStyleSheet(u"/* --- Global & Base Widget --- */\n"
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
        self.verticalLayout_2 = QVBoxLayout(CalibrateWindow)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.headerLabel = QLabel(CalibrateWindow)
        self.headerLabel.setObjectName(u"headerLabel")
        self.headerLabel.setMinimumSize(QSize(270, 30))

        self.verticalLayout_2.addWidget(self.headerLabel)

        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.label = QLabel(CalibrateWindow)
        self.label.setObjectName(u"label")
        self.label.setMinimumSize(QSize(360, 40))
        self.label.setTextFormat(Qt.TextFormat.MarkdownText)
        self.label.setWordWrap(True)

        self.verticalLayout.addWidget(self.label)

        self.checkBox = QCheckBox(CalibrateWindow)
        self.checkBox.setObjectName(u"checkBox")
        self.checkBox.setMinimumSize(QSize(360, 40))

        self.verticalLayout.addWidget(self.checkBox)

        self.label_2 = QLabel(CalibrateWindow)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setMinimumSize(QSize(360, 40))
        self.label_2.setTextFormat(Qt.TextFormat.MarkdownText)
        self.label_2.setWordWrap(True)

        self.verticalLayout.addWidget(self.label_2)

        self.checkBox_2 = QCheckBox(CalibrateWindow)
        self.checkBox_2.setObjectName(u"checkBox_2")
        self.checkBox_2.setMinimumSize(QSize(360, 40))

        self.verticalLayout.addWidget(self.checkBox_2)

        self.label_3 = QLabel(CalibrateWindow)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setMinimumSize(QSize(360, 40))
        self.label_3.setTextFormat(Qt.TextFormat.MarkdownText)
        self.label_3.setWordWrap(True)

        self.verticalLayout.addWidget(self.label_3)

        self.checkBox_3 = QCheckBox(CalibrateWindow)
        self.checkBox_3.setObjectName(u"checkBox_3")
        self.checkBox_3.setMinimumSize(QSize(360, 40))

        self.verticalLayout.addWidget(self.checkBox_3)


        self.verticalLayout_2.addLayout(self.verticalLayout)

        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer)

        self.calibrateButton = QPushButton(CalibrateWindow)
        self.calibrateButton.setObjectName(u"calibrateButton")
        self.calibrateButton.setMinimumSize(QSize(220, 30))

        self.horizontalLayout.addWidget(self.calibrateButton)

        self.horizontalSpacer_2 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer_2)


        self.verticalLayout_2.addLayout(self.horizontalLayout)


        self.retranslateUi(CalibrateWindow)

        QMetaObject.connectSlotsByName(CalibrateWindow)
    # setupUi

    def retranslateUi(self, CalibrateWindow):
        CalibrateWindow.setWindowTitle(QCoreApplication.translate("CalibrateWindow", u"Form", None))
        self.headerLabel.setText(QCoreApplication.translate("CalibrateWindow", u"Calibration", None))
        self.label.setText(QCoreApplication.translate("CalibrateWindow", u"Confirm that the arms are approximately in the desired configuration (shown on the right)", None))
        self.checkBox.setText(QCoreApplication.translate("CalibrateWindow", u"Confirm", None))
        self.label_2.setText(QCoreApplication.translate("CalibrateWindow", u"Confirm that there are no dishes in the transfer area", None))
        self.checkBox_2.setText(QCoreApplication.translate("CalibrateWindow", u"Confirm", None))
        self.label_3.setText(QCoreApplication.translate("CalibrateWindow", u"Warning! Once calibration starts, the only way to stop is via the emergency stop!", None))
        self.checkBox_3.setText(QCoreApplication.translate("CalibrateWindow", u"I understand", None))
        self.calibrateButton.setText(QCoreApplication.translate("CalibrateWindow", u"Calibrate", None))
    # retranslateUi

