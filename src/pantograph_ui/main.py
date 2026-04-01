import sys
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget
from ui_calibrate_window import Ui_Form
from ui_main_window import Ui_MainWindow


class CalibrateWindow(QWidget, Ui_Form):
    def __init__(self):
        super().__init__()
        self.setupUi(self)


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.calibrate_window = None
        self.calibrateButton.clicked.connect(self.on_click)

    def on_click(self):
        if self.calibrate_window is None:
            self.calibrate_window = CalibrateWindow()
        self.calibrate_window.show()
        self.calibrate_window.raise_()
        self.calibrate_window.activateWindow()

app = QApplication(sys.argv)
window = MainWindow()
window.show()
sys.exit(app.exec())
