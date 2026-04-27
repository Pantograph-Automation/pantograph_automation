# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'CalibrateWindow.ui'
##
## Created by: Qt User Interface Compiler version 6.11.0
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
from PySide6.QtWidgets import (QApplication, QCheckBox, QFrame, QLabel,
    QPushButton, QSizePolicy, QSpacerItem, QVBoxLayout,
    QWidget)

class Ui_CalibrateWindow(object):
    def setupUi(self, CalibrateWindow):
        if not CalibrateWindow.objectName():
            CalibrateWindow.setObjectName(u"CalibrateWindow")
        CalibrateWindow.resize(430, 420)
        CalibrateWindow.setMinimumSize(QSize(400, 390))
        CalibrateWindow.setStyleSheet(u"QWidget {\n"
"    background-color: #f6f8fb;\n"
"    color: #1f2937;\n"
"    font-family: \"Segoe UI\", \"Helvetica Neue\", Arial, sans-serif;\n"
"    font-size: 14px;\n"
"}\n"
"\n"
"QFrame#contentFrame {\n"
"    background-color: #ffffff;\n"
"    border: 1px solid #d9e2ec;\n"
"    border-radius: 8px;\n"
"}\n"
"\n"
"QLabel {\n"
"    background: transparent;\n"
"    color: #475569;\n"
"}\n"
"\n"
"QLabel#headerLabel {\n"
"    color: #0f172a;\n"
"    font-size: 24px;\n"
"    font-weight: 700;\n"
"}\n"
"\n"
"QLabel#warningLabel {\n"
"    color: #991b1b;\n"
"    font-weight: 700;\n"
"}\n"
"\n"
"QCheckBox {\n"
"    color: #334155;\n"
"    spacing: 10px;\n"
"}\n"
"\n"
"QCheckBox::indicator {\n"
"    width: 18px;\n"
"    height: 18px;\n"
"    border: 1px solid #cbd5e1;\n"
"    border-radius: 4px;\n"
"    background-color: #ffffff;\n"
"}\n"
"\n"
"QCheckBox::indicator:hover {\n"
"    border: 1px solid #2563eb;\n"
"}\n"
"\n"
"QCheckBox::indicator:checked {\n"
"    background-color: #2563eb;\n"
"    border: 1px solid #256"
                        "3eb;\n"
"}\n"
"\n"
"QPushButton#calibrateButton {\n"
"    background-color: #1d4ed8;\n"
"    color: #ffffff;\n"
"    border: none;\n"
"    border-radius: 6px;\n"
"    padding: 11px 18px;\n"
"    font-weight: 700;\n"
"}\n"
"\n"
"QPushButton#calibrateButton:hover {\n"
"    background-color: #1e40af;\n"
"}\n"
"\n"
"QPushButton#calibrateButton:disabled {\n"
"    background-color: #e2e8f0;\n"
"    color: #94a3b8;\n"
"}")
        self.outerLayout = QVBoxLayout(CalibrateWindow)
        self.outerLayout.setObjectName(u"outerLayout")
        self.outerLayout.setContentsMargins(20, 20, 20, 20)
        self.contentFrame = QFrame(CalibrateWindow)
        self.contentFrame.setObjectName(u"contentFrame")
        self.contentFrame.setFrameShape(QFrame.Shape.StyledPanel)
        self.verticalLayout_2 = QVBoxLayout(self.contentFrame)
        self.verticalLayout_2.setSpacing(14)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout_2.setContentsMargins(20, 18, 20, 18)
        self.headerLabel = QLabel(self.contentFrame)
        self.headerLabel.setObjectName(u"headerLabel")

        self.verticalLayout_2.addWidget(self.headerLabel)

        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setSpacing(10)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.label = QLabel(self.contentFrame)
        self.label.setObjectName(u"label")
        self.label.setWordWrap(True)

        self.verticalLayout.addWidget(self.label)

        self.checkBox = QCheckBox(self.contentFrame)
        self.checkBox.setObjectName(u"checkBox")
        self.checkBox.setMinimumSize(QSize(0, 34))

        self.verticalLayout.addWidget(self.checkBox)

        self.label_2 = QLabel(self.contentFrame)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setWordWrap(True)

        self.verticalLayout.addWidget(self.label_2)

        self.checkBox2 = QCheckBox(self.contentFrame)
        self.checkBox2.setObjectName(u"checkBox2")
        self.checkBox2.setMinimumSize(QSize(0, 34))

        self.verticalLayout.addWidget(self.checkBox2)

        self.warningLabel = QLabel(self.contentFrame)
        self.warningLabel.setObjectName(u"warningLabel")
        self.warningLabel.setWordWrap(True)

        self.verticalLayout.addWidget(self.warningLabel)

        self.checkBox3 = QCheckBox(self.contentFrame)
        self.checkBox3.setObjectName(u"checkBox3")
        self.checkBox3.setMinimumSize(QSize(0, 34))

        self.verticalLayout.addWidget(self.checkBox3)


        self.verticalLayout_2.addLayout(self.verticalLayout)

        self.verticalSpacer = QSpacerItem(20, 20, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_2.addItem(self.verticalSpacer)

        self.calibrateButton = QPushButton(self.contentFrame)
        self.calibrateButton.setObjectName(u"calibrateButton")
        self.calibrateButton.setMinimumSize(QSize(0, 44))

        self.verticalLayout_2.addWidget(self.calibrateButton)


        self.outerLayout.addWidget(self.contentFrame)


        self.retranslateUi(CalibrateWindow)

        QMetaObject.connectSlotsByName(CalibrateWindow)
    # setupUi

    def retranslateUi(self, CalibrateWindow):
        CalibrateWindow.setWindowTitle(QCoreApplication.translate("CalibrateWindow", u"Calibration", None))
        self.headerLabel.setText(QCoreApplication.translate("CalibrateWindow", u"Calibration", None))
        self.label.setText(QCoreApplication.translate("CalibrateWindow", u"Confirm the arms are approximately in the desired start configuration.", None))
        self.checkBox.setText(QCoreApplication.translate("CalibrateWindow", u"Arms are positioned", None))
        self.label_2.setText(QCoreApplication.translate("CalibrateWindow", u"Confirm there are no dishes in the transfer area.", None))
        self.checkBox2.setText(QCoreApplication.translate("CalibrateWindow", u"Transfer area is clear", None))
        self.warningLabel.setText(QCoreApplication.translate("CalibrateWindow", u"Once calibration starts, use the emergency stop to halt motion.", None))
        self.checkBox3.setText(QCoreApplication.translate("CalibrateWindow", u"I understand", None))
        self.calibrateButton.setText(QCoreApplication.translate("CalibrateWindow", u"Calibrate", None))
    # retranslateUi

