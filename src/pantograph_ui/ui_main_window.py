# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'MainWindow.ui'
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
from PySide6.QtWidgets import (QApplication, QComboBox, QFrame, QHBoxLayout,
    QLabel, QMainWindow, QProgressBar, QPushButton,
    QSizePolicy, QSpacerItem, QStatusBar, QVBoxLayout,
    QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(860, 560)
        MainWindow.setMinimumSize(QSize(780, 520))
        MainWindow.setStyleSheet(u"QWidget {\n"
"    background-color: #f6f8fb;\n"
"    color: #1f2937;\n"
"    font-family: \"Segoe UI\", \"Helvetica Neue\", Arial, sans-serif;\n"
"    font-size: 14px;\n"
"}\n"
"\n"
"QFrame#headerFrame,\n"
"QFrame#setupFrame,\n"
"QFrame#progressFrame,\n"
"QFrame#metricsFrame,\n"
"QFrame#poseFrame {\n"
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
"QLabel#titleLabel {\n"
"    color: #0f172a;\n"
"    font-size: 28px;\n"
"    font-weight: 700;\n"
"}\n"
"\n"
"QLabel#sectionLabel,\n"
"QLabel#cycleTitleLabel {\n"
"    color: #111827;\n"
"    font-size: 17px;\n"
"    font-weight: 700;\n"
"}\n"
"\n"
"QLabel#readinessLabel {\n"
"    color: #ffffff;\n"
"    border-radius: 16px;\n"
"    padding: 6px 14px;\n"
"    font-weight: 700;\n"
"}\n"
"\n"
"QLabel#stateLabel {\n"
"    color: #334155;\n"
"    font-size: 16px;\n"
"    font-weight: 700;\n"
"}\n"
"\n"
"QLabel#detailLabel,\n"
"QL"
                        "abel#poseValueLabel {\n"
"    color: #64748b;\n"
"}\n"
"\n"
"QLabel#metricValueLabel,\n"
"QLabel#detectedValueLabel,\n"
"QLabel#transferredValueLabel,\n"
"QLabel#nextBatchValueLabel {\n"
"    color: #0f172a;\n"
"    font-size: 24px;\n"
"    font-weight: 700;\n"
"}\n"
"\n"
"QComboBox {\n"
"    background-color: #ffffff;\n"
"    border: 1px solid #cbd5e1;\n"
"    border-radius: 6px;\n"
"    padding: 8px 12px;\n"
"    min-height: 22px;\n"
"}\n"
"\n"
"QComboBox:disabled {\n"
"    background-color: #eef2f7;\n"
"    color: #94a3b8;\n"
"}\n"
"\n"
"QProgressBar {\n"
"    background-color: #e8edf4;\n"
"    border: none;\n"
"    border-radius: 6px;\n"
"    height: 12px;\n"
"    text-align: center;\n"
"    color: transparent;\n"
"}\n"
"\n"
"QProgressBar::chunk {\n"
"    background-color: #2563eb;\n"
"    border-radius: 6px;\n"
"}\n"
"\n"
"QPushButton {\n"
"    border: none;\n"
"    border-radius: 6px;\n"
"    padding: 11px 18px;\n"
"    font-weight: 700;\n"
"}\n"
"\n"
"QPushButton#runButton {\n"
"    background-color: #1"
                        "d4ed8;\n"
"    color: #ffffff;\n"
"}\n"
"\n"
"QPushButton#runButton:hover {\n"
"    background-color: #1e40af;\n"
"}\n"
"\n"
"QPushButton#calibrateButton {\n"
"    background-color: #ffffff;\n"
"    color: #1d4ed8;\n"
"    border: 1px solid #93c5fd;\n"
"}\n"
"\n"
"QPushButton#calibrateButton:hover {\n"
"    background-color: #eff6ff;\n"
"}\n"
"\n"
"QPushButton:disabled {\n"
"    background-color: #e2e8f0;\n"
"    color: #94a3b8;\n"
"    border: 1px solid #dbe3ee;\n"
"}")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.mainLayout = QVBoxLayout(self.centralwidget)
        self.mainLayout.setSpacing(16)
        self.mainLayout.setObjectName(u"mainLayout")
        self.mainLayout.setContentsMargins(24, 24, 24, 18)
        self.headerFrame = QFrame(self.centralwidget)
        self.headerFrame.setObjectName(u"headerFrame")
        self.headerFrame.setFrameShape(QFrame.Shape.StyledPanel)
        self.headerLayout = QHBoxLayout(self.headerFrame)
        self.headerLayout.setSpacing(18)
        self.headerLayout.setObjectName(u"headerLayout")
        self.headerLayout.setContentsMargins(18, 16, 18, 16)
        self.titleLayout = QVBoxLayout()
        self.titleLayout.setSpacing(4)
        self.titleLayout.setObjectName(u"titleLayout")
        self.titleLabel = QLabel(self.headerFrame)
        self.titleLabel.setObjectName(u"titleLabel")

        self.titleLayout.addWidget(self.titleLabel)

        self.detailLabel = QLabel(self.headerFrame)
        self.detailLabel.setObjectName(u"detailLabel")
        self.detailLabel.setWordWrap(True)

        self.titleLayout.addWidget(self.detailLabel)


        self.headerLayout.addLayout(self.titleLayout)

        self.headerSpacer = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.headerLayout.addItem(self.headerSpacer)

        self.statusLayout = QVBoxLayout()
        self.statusLayout.setSpacing(8)
        self.statusLayout.setObjectName(u"statusLayout")
        self.readinessLabel = QLabel(self.headerFrame)
        self.readinessLabel.setObjectName(u"readinessLabel")
        self.readinessLabel.setMinimumSize(QSize(126, 34))
        self.readinessLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.statusLayout.addWidget(self.readinessLabel)

        self.stateLabel = QLabel(self.headerFrame)
        self.stateLabel.setObjectName(u"stateLabel")
        self.stateLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.statusLayout.addWidget(self.stateLabel)


        self.headerLayout.addLayout(self.statusLayout)


        self.mainLayout.addWidget(self.headerFrame)

        self.contentLayout = QHBoxLayout()
        self.contentLayout.setSpacing(16)
        self.contentLayout.setObjectName(u"contentLayout")
        self.leftColumnLayout = QVBoxLayout()
        self.leftColumnLayout.setSpacing(16)
        self.leftColumnLayout.setObjectName(u"leftColumnLayout")
        self.setupFrame = QFrame(self.centralwidget)
        self.setupFrame.setObjectName(u"setupFrame")
        self.setupFrame.setFrameShape(QFrame.Shape.StyledPanel)
        self.setupLayout = QVBoxLayout(self.setupFrame)
        self.setupLayout.setSpacing(12)
        self.setupLayout.setObjectName(u"setupLayout")
        self.setupLayout.setContentsMargins(18, 16, 18, 16)
        self.cycleTitleLabel = QLabel(self.setupFrame)
        self.cycleTitleLabel.setObjectName(u"cycleTitleLabel")

        self.setupLayout.addWidget(self.cycleTitleLabel)

        self.ratioLayout = QHBoxLayout()
        self.ratioLayout.setSpacing(12)
        self.ratioLayout.setObjectName(u"ratioLayout")
        self.ratioLabel = QLabel(self.setupFrame)
        self.ratioLabel.setObjectName(u"ratioLabel")

        self.ratioLayout.addWidget(self.ratioLabel)

        self.ratioSpacer = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.ratioLayout.addItem(self.ratioSpacer)

        self.ratioComboBox = QComboBox(self.setupFrame)
        self.ratioComboBox.addItem("")
        self.ratioComboBox.addItem("")
        self.ratioComboBox.setObjectName(u"ratioComboBox")
        self.ratioComboBox.setMinimumSize(QSize(128, 38))

        self.ratioLayout.addWidget(self.ratioComboBox)


        self.setupLayout.addLayout(self.ratioLayout)


        self.leftColumnLayout.addWidget(self.setupFrame)

        self.progressFrame = QFrame(self.centralwidget)
        self.progressFrame.setObjectName(u"progressFrame")
        self.progressFrame.setFrameShape(QFrame.Shape.StyledPanel)
        self.progressLayout = QVBoxLayout(self.progressFrame)
        self.progressLayout.setSpacing(14)
        self.progressLayout.setObjectName(u"progressLayout")
        self.progressLayout.setContentsMargins(18, 16, 18, 16)
        self.sectionLabel = QLabel(self.progressFrame)
        self.sectionLabel.setObjectName(u"sectionLabel")

        self.progressLayout.addWidget(self.sectionLabel)

        self.sourceLabelLayout = QHBoxLayout()
        self.sourceLabelLayout.setObjectName(u"sourceLabelLayout")
        self.sourceLabel = QLabel(self.progressFrame)
        self.sourceLabel.setObjectName(u"sourceLabel")

        self.sourceLabelLayout.addWidget(self.sourceLabel)

        self.sourceSpacer = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.sourceLabelLayout.addItem(self.sourceSpacer)

        self.sourceCountLabel = QLabel(self.progressFrame)
        self.sourceCountLabel.setObjectName(u"sourceCountLabel")

        self.sourceLabelLayout.addWidget(self.sourceCountLabel)


        self.progressLayout.addLayout(self.sourceLabelLayout)

        self.sourceProgressBar = QProgressBar(self.progressFrame)
        self.sourceProgressBar.setObjectName(u"sourceProgressBar")
        self.sourceProgressBar.setMaximum(45)
        self.sourceProgressBar.setValue(0)
        self.sourceProgressBar.setTextVisible(False)

        self.progressLayout.addWidget(self.sourceProgressBar)

        self.destinationLabelLayout = QHBoxLayout()
        self.destinationLabelLayout.setObjectName(u"destinationLabelLayout")
        self.destinationLabel = QLabel(self.progressFrame)
        self.destinationLabel.setObjectName(u"destinationLabel")

        self.destinationLabelLayout.addWidget(self.destinationLabel)

        self.destinationSpacer = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.destinationLabelLayout.addItem(self.destinationSpacer)

        self.destinationCountLabel = QLabel(self.progressFrame)
        self.destinationCountLabel.setObjectName(u"destinationCountLabel")

        self.destinationLabelLayout.addWidget(self.destinationCountLabel)


        self.progressLayout.addLayout(self.destinationLabelLayout)

        self.destinationProgressBar = QProgressBar(self.progressFrame)
        self.destinationProgressBar.setObjectName(u"destinationProgressBar")
        self.destinationProgressBar.setMaximum(30)
        self.destinationProgressBar.setValue(0)
        self.destinationProgressBar.setTextVisible(False)

        self.progressLayout.addWidget(self.destinationProgressBar)


        self.leftColumnLayout.addWidget(self.progressFrame)

        self.poseFrame = QFrame(self.centralwidget)
        self.poseFrame.setObjectName(u"poseFrame")
        self.poseFrame.setFrameShape(QFrame.Shape.StyledPanel)
        self.poseLayout = QVBoxLayout(self.poseFrame)
        self.poseLayout.setObjectName(u"poseLayout")
        self.poseLayout.setContentsMargins(18, 14, 18, 14)
        self.poseValueLabel = QLabel(self.poseFrame)
        self.poseValueLabel.setObjectName(u"poseValueLabel")

        self.poseLayout.addWidget(self.poseValueLabel)


        self.leftColumnLayout.addWidget(self.poseFrame)


        self.contentLayout.addLayout(self.leftColumnLayout)

        self.metricsFrame = QFrame(self.centralwidget)
        self.metricsFrame.setObjectName(u"metricsFrame")
        self.metricsFrame.setMinimumSize(QSize(260, 0))
        self.metricsFrame.setFrameShape(QFrame.Shape.StyledPanel)
        self.metricsLayout = QVBoxLayout(self.metricsFrame)
        self.metricsLayout.setSpacing(18)
        self.metricsLayout.setObjectName(u"metricsLayout")
        self.metricsLayout.setContentsMargins(18, 16, 18, 16)
        self.metricsTitleLabel = QLabel(self.metricsFrame)
        self.metricsTitleLabel.setObjectName(u"metricsTitleLabel")

        self.metricsLayout.addWidget(self.metricsTitleLabel)

        self.detectedLayout = QVBoxLayout()
        self.detectedLayout.setSpacing(3)
        self.detectedLayout.setObjectName(u"detectedLayout")
        self.detectedValueLabel = QLabel(self.metricsFrame)
        self.detectedValueLabel.setObjectName(u"detectedValueLabel")

        self.detectedLayout.addWidget(self.detectedValueLabel)

        self.detectedLabel = QLabel(self.metricsFrame)
        self.detectedLabel.setObjectName(u"detectedLabel")

        self.detectedLayout.addWidget(self.detectedLabel)


        self.metricsLayout.addLayout(self.detectedLayout)

        self.transferredLayout = QVBoxLayout()
        self.transferredLayout.setSpacing(3)
        self.transferredLayout.setObjectName(u"transferredLayout")
        self.transferredValueLabel = QLabel(self.metricsFrame)
        self.transferredValueLabel.setObjectName(u"transferredValueLabel")

        self.transferredLayout.addWidget(self.transferredValueLabel)

        self.transferredLabel = QLabel(self.metricsFrame)
        self.transferredLabel.setObjectName(u"transferredLabel")

        self.transferredLayout.addWidget(self.transferredLabel)


        self.metricsLayout.addLayout(self.transferredLayout)

        self.nextBatchLayout = QVBoxLayout()
        self.nextBatchLayout.setSpacing(3)
        self.nextBatchLayout.setObjectName(u"nextBatchLayout")
        self.nextBatchValueLabel = QLabel(self.metricsFrame)
        self.nextBatchValueLabel.setObjectName(u"nextBatchValueLabel")

        self.nextBatchLayout.addWidget(self.nextBatchValueLabel)

        self.nextBatchLabel = QLabel(self.metricsFrame)
        self.nextBatchLabel.setObjectName(u"nextBatchLabel")

        self.nextBatchLayout.addWidget(self.nextBatchLabel)


        self.metricsLayout.addLayout(self.nextBatchLayout)

        self.metricsSpacer = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.metricsLayout.addItem(self.metricsSpacer)

        self.calibrateButton = QPushButton(self.metricsFrame)
        self.calibrateButton.setObjectName(u"calibrateButton")
        self.calibrateButton.setMinimumSize(QSize(0, 42))

        self.metricsLayout.addWidget(self.calibrateButton)

        self.runButton = QPushButton(self.metricsFrame)
        self.runButton.setObjectName(u"runButton")
        self.runButton.setMinimumSize(QSize(0, 46))

        self.metricsLayout.addWidget(self.runButton)


        self.contentLayout.addWidget(self.metricsFrame)


        self.mainLayout.addLayout(self.contentLayout)

        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"Pantograph Full Cycle", None))
        self.titleLabel.setText(QCoreApplication.translate("MainWindow", u"Full Cycle Transfer", None))
        self.detailLabel.setText(QCoreApplication.translate("MainWindow", u"Idle", None))
        self.readinessLabel.setText(QCoreApplication.translate("MainWindow", u"Not Ready", None))
        self.stateLabel.setText(QCoreApplication.translate("MainWindow", u"Idle", None))
        self.cycleTitleLabel.setText(QCoreApplication.translate("MainWindow", u"Cycle Setup", None))
        self.ratioLabel.setText(QCoreApplication.translate("MainWindow", u"Incoming to outgoing", None))
        self.ratioComboBox.setItemText(0, QCoreApplication.translate("MainWindow", u"45:30", None))
        self.ratioComboBox.setItemText(1, QCoreApplication.translate("MainWindow", u"30:20", None))

        self.sectionLabel.setText(QCoreApplication.translate("MainWindow", u"Dish Progress", None))
        self.sourceLabel.setText(QCoreApplication.translate("MainWindow", u"Source used", None))
        self.sourceCountLabel.setText(QCoreApplication.translate("MainWindow", u"0 / 45", None))
        self.destinationLabel.setText(QCoreApplication.translate("MainWindow", u"Destination filled", None))
        self.destinationCountLabel.setText(QCoreApplication.translate("MainWindow", u"0 / 30", None))
        self.poseValueLabel.setText(QCoreApplication.translate("MainWindow", u"Pose x=0.000 m, y=0.000 m, z=0.000 m", None))
        self.metricsTitleLabel.setText(QCoreApplication.translate("MainWindow", u"Run Metrics", None))
        self.detectedValueLabel.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.detectedLabel.setText(QCoreApplication.translate("MainWindow", u"Detected clusters", None))
        self.transferredValueLabel.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.transferredLabel.setText(QCoreApplication.translate("MainWindow", u"Transferred this run", None))
        self.nextBatchValueLabel.setText(QCoreApplication.translate("MainWindow", u"30", None))
        self.nextBatchLabel.setText(QCoreApplication.translate("MainWindow", u"Next planned batch", None))
        self.calibrateButton.setText(QCoreApplication.translate("MainWindow", u"Calibrate", None))
        self.runButton.setText(QCoreApplication.translate("MainWindow", u"Run Full Cycle", None))
    # retranslateUi

