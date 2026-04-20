from __future__ import annotations

import math
import sys
from pathlib import Path

from PySide6.QtWidgets import QApplication, QMainWindow, QWidget

SRC_ROOT = Path(__file__).resolve().parents[1]
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from pantograph_control.macros import ControllerStatus, Controller, Pose
from pantograph_ui.ui_calibrate_window import Ui_CalibrateWindow
from pantograph_ui.ui_main_window import Ui_MainWindow
from pantograph_ui.ui_manual_window import Ui_ManualWindow


class ManualWindow(QWidget, Ui_ManualWindow):
    def __init__(self, controller: Controller):
        super().__init__()
        self.controller = controller
        self.setupUi(self)
        self.setWindowTitle("Manual Control")
        self.horizontalSlider.setValue(473)
        self._update_step_label()
        self._set_controls_enabled(False)

        self.horizontalSlider.valueChanged.connect(self._update_step_label)
        self.posXButton.clicked.connect(lambda: self._jog(dx=self._step_size()))
        self.negXButton.clicked.connect(lambda: self._jog(dx=-self._step_size()))
        self.posYButton.clicked.connect(lambda: self._jog(dy=self._step_size()))
        self.negYButton.clicked.connect(lambda: self._jog(dy=-self._step_size()))
        self.posZButton.clicked.connect(lambda: self._jog(dz=self._step_size()))
        self.negZButton.clicked.connect(lambda: self._jog(dz=-self._step_size()))

        self.controller.pose_changed.connect(self._update_pose)
        self.controller.status_changed.connect(self._update_status)

    def _step_size(self) -> float:
        return self.controller.slider_value_to_step(
            self.horizontalSlider.value(),
            self.horizontalSlider.maximum(),
        )

    def _update_step_label(self) -> None:
        self.yLabel_2.setText(f"{self._step_size():.3f} m")

    def _set_controls_enabled(self, enabled: bool) -> None:
        for widget in (
            self.posXButton,
            self.negXButton,
            self.posYButton,
            self.negYButton,
            self.posZButton,
            self.negZButton,
            self.horizontalSlider,
        ):
            widget.setEnabled(enabled)

    def _jog(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> None:
        self.controller.manual_jog_async(dx=dx, dy=dy, dz=dz)

    def _update_pose(self, pose: Pose) -> None:
        self.xLabel.setText(f"{pose.point[0]:.3f} m")
        self.yLabel.setText(f"{pose.point[1]:.3f} m")
        self.zLabel.setText(f"{pose.point[2]:.3f} m")
        self.j1Label.setText(f"{math.degrees(pose.theta1):.1f} deg")
        self.j2Label.setText(f"{math.degrees(pose.theta5):.1f} deg")

    def _update_status(self, status: ControllerStatus) -> None:
        enabled = status.calibrated and not status.busy
        self._set_controls_enabled(enabled)


class CalibrateWindow(QWidget, Ui_CalibrateWindow):
    def __init__(self, controller: Controller):
        super().__init__()
        self.controller = controller
        self.setupUi(self)
        self.setWindowTitle("Calibration")

        self.checkBox.stateChanged.connect(self._refresh_button_state)
        self.checkBox2.stateChanged.connect(self._refresh_button_state)
        self.checkBox3.stateChanged.connect(self._refresh_button_state)
        self.calibrateButton.clicked.connect(self._start_calibration)
        self.controller.status_changed.connect(self._update_status)
        self._refresh_button_state()

    def _all_confirmed(self) -> bool:
        return self.checkBox.isChecked() and self.checkBox2.isChecked() and self.checkBox3.isChecked()

    def _refresh_button_state(self) -> None:
        self.calibrateButton.setEnabled(self._all_confirmed())

    def _start_calibration(self) -> None:
        self.controller.calibrate_async()

    def _update_status(self, status: ControllerStatus) -> None:
        if status.busy:
            self.calibrateButton.setEnabled(False)
            return
        self._refresh_button_state()


class MainWindow(QMainWindow, Ui_MainWindow):
    STATUS_COLORS = {
        "idle": "#666666",
        "calibrated": "#1d7d4f",
        "running": "#a85d00",
        "capturing": "#a85d00",
        "calibrating": "#a85d00",
        "manual": "#1a1a1a",
        "complete": "#1d7d4f",
        "error": "#a02323",
    }

    def __init__(self, controller: Controller):
        super().__init__()
        self.controller = controller
        self.setupUi(self)
        self.setWindowTitle("Pantograph Prototype")
        self.calibrate_window: CalibrateWindow | None = None
        self.manual_window: ManualWindow | None = None

        self._configure_v1_inputs()

        self.calibrateButton.clicked.connect(self.launch_calibrate)
        self.manualButton.clicked.connect(self.launch_manual)
        self.runButton.clicked.connect(self._start_run)

        self.controller.status_changed.connect(self._update_status)
        self.controller.pose_changed.connect(self._update_pose)
        self.controller.log_message.connect(self.statusbar.showMessage)
        self.controller.emit_initial_state()

    def _configure_v1_inputs(self) -> None:
        self.lineEditA.setText(str(self.controller.config.nominal_clusters))
        self.lineEditB.setText(str(self.controller.config.outgoing_clusters))
        self.lineEditC.setText("0")
        self.lineEditD.setText("0")

        self.comboBoxA.addItems(["Source"])
        self.comboBoxB.addItems(["Destination"])
        self.comboBoxC.addItems(["Inactive"])
        self.comboBoxD.addItems(["Inactive"])

        for widget in (
            self.lineEditA,
            self.lineEditB,
            self.lineEditC,
            self.lineEditD,
            self.comboBoxA,
            self.comboBoxB,
            self.comboBoxC,
            self.comboBoxD,
        ):
            widget.setEnabled(False)

    def launch_calibrate(self) -> None:
        if self.calibrate_window is None:
            self.calibrate_window = CalibrateWindow(self.controller)
        self.calibrate_window.show()
        self.calibrate_window.raise_()
        self.calibrate_window.activateWindow()

    def launch_manual(self) -> None:
        if self.manual_window is None:
            self.manual_window = ManualWindow(self.controller)
        self.manual_window.show()
        self.manual_window.raise_()
        self.manual_window.activateWindow()

    def _start_run(self) -> None:
        self.controller.run_transfer_async()

    def _update_pose(self, pose: Pose) -> None:
        self.statusbar.showMessage(
            f"Pose x={pose.point[0]:.3f} m, y={pose.point[1]:.3f} m, z={pose.point[2]:.3f} m"
        )

    def _update_status(self, status: ControllerStatus) -> None:
        state_text = status.state.capitalize()
        if status.calibrated and status.state not in {"error", "running", "capturing", "calibrating"}:
            top_text = "Calibrated"
            top_color = self.STATUS_COLORS["calibrated"]
        elif status.state == "error":
            top_text = "Error"
            top_color = self.STATUS_COLORS["error"]
        else:
            top_text = "Not Ready"
            top_color = self.STATUS_COLORS.get(status.state, self.STATUS_COLORS["idle"])

        bottom_text = state_text if status.state != "idle" else "Idle"
        bottom_color = self.STATUS_COLORS.get(status.state, self.STATUS_COLORS["idle"])

        self._set_status_badge(self.topStatusLabel, top_text, top_color)
        self._set_status_badge(self.bottomStatusLabel, bottom_text, bottom_color)
        self.statusbar.showMessage(status.detail)

        run_enabled = status.calibrated and not status.busy
        self.runButton.setEnabled(run_enabled)
        self.calibrateButton.setEnabled(not status.busy)
        self.manualButton.setEnabled(not status.busy)

        self.dishLabelA.setText(f"Dish A\nDetected {status.detected_count}")
        self.dishLabelB.setText(f"Dish B\nPlaced {status.transferred_count}")
        self.dishLabelC.setText("Dish C\nInactive")
        self.dishLabelD.setText("Dish D\nInactive")

    def _set_status_badge(self, label, text: str, color: str) -> None:
        label.setText(text)
        label.setStyleSheet(
            "QLabel {"
            f"background-color: {color};"
            "color: white;"
            "border-radius: 20px;"
            "font-weight: 600;"
            "padding: 4px 8px;"
            "}"
        )


def main() -> int:
    app = QApplication(sys.argv)
    controller = Controller()
    window = MainWindow(controller)
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
