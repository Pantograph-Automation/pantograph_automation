import sys
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget
from ui_calibrate_window import Ui_CalibrateWindow
from ui_manual_window import Ui_ManualWindow
from ui_main_window import Ui_MainWindow

class ManualWindow(QWidget, Ui_ManualWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

class CalibrateWindow(QWidget, Ui_CalibrateWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.calibrate_window = None
        self.manual_window = None
        self.calibrateButton.clicked.connect(self.launch_calibrate)
        self.manualButton.clicked.connect(self.launch_manual)

    def launch_calibrate(self):
        if self.calibrate_window is None:
            self.calibrate_window = CalibrateWindow()
        self.calibrate_window.show()
        self.calibrate_window.raise_()
        self.calibrate_window.activateWindow()

    def launch_manual(self):
        if self.manual_window is None:
            self.manual_window = ManualWindow()
        self.manual_window.show()
        self.manual_window.raise_()
        self.manual_window.activateWindow()

app = QApplication(sys.argv)
window = MainWindow()
window.show()
sys.exit(app.exec())
