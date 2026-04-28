from __future__ import annotations

import sys
from pathlib import Path

from PySide6.QtWidgets import QApplication, QMainWindow, QWidget

SRC_ROOT = Path(__file__).resolve().parents[1]
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from pantograph_control.macros import ControllerStatus, Controller, Pose, TransferConfig
from pantograph_ui.ui_calibrate_window import Ui_CalibrateWindow
from pantograph_ui.ui_main_window import Ui_MainWindow

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
    CYCLE_RATIOS = {
        "45:30": (45, 30),
        "30:20": (30, 20),
    }
    STATUS_COLORS = {
        "idle": "#64748b",
        "calibrated": "#15803d",
        "running": "#b45309",
        "capturing": "#b45309",
        "calibrating": "#b45309",
        "manual": "#334155",
        "complete": "#15803d",
        "error": "#b91c1c",
    }

    def __init__(self, controller: Controller | None = None):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Pantograph Full Cycle")
        self.controller: Controller | None = None
        self.calibrate_window: CalibrateWindow | None = None

        self.calibrateButton.clicked.connect(self.launch_calibrate)
        self.runButton.clicked.connect(self._start_run)
        self.ratioComboBox.currentTextChanged.connect(self._handle_ratio_changed)

        if controller is None:
            controller = self._build_controller_for_ratio(self.ratioComboBox.currentText())
        else:
            self._select_ratio_for_controller(controller)

        self._replace_controller(controller)

    def _build_controller_for_ratio(self, ratio: str) -> Controller:
        nominal_clusters, outgoing_clusters = self.CYCLE_RATIOS.get(
            ratio,
            self.CYCLE_RATIOS["45:30"],
        )
        return Controller(
            TransferConfig(
                nominal_clusters=nominal_clusters,
                outgoing_clusters=outgoing_clusters,
            )
        )

    def _select_ratio_for_controller(self, controller: Controller) -> None:
        ratio = f"{controller.config.nominal_clusters}:{controller.config.outgoing_clusters}"
        if ratio not in self.CYCLE_RATIOS:
            ratio = "45:30"
        self.ratioComboBox.blockSignals(True)
        self.ratioComboBox.setCurrentText(ratio)
        self.ratioComboBox.blockSignals(False)

    def _replace_controller(self, controller: Controller) -> None:
        if self.controller is not None:
            self._disconnect_controller(self.controller)
        if self.calibrate_window is not None:
            self.calibrate_window.close()
            self.calibrate_window = None

        self.controller = controller
        self.controller.status_changed.connect(self._update_status)
        self.controller.pose_changed.connect(self._update_pose)
        self.controller.clusters_changed.connect(self._update_clusters)
        self.controller.log_message.connect(self._show_message)
        self.controller.emit_initial_state()

    def _disconnect_controller(self, controller: Controller) -> None:
        for signal, slot in (
            (controller.status_changed, self._update_status),
            (controller.pose_changed, self._update_pose),
            (controller.clusters_changed, self._update_clusters),
            (controller.log_message, self._show_message),
        ):
            try:
                signal.disconnect(slot)
            except (RuntimeError, TypeError):
                pass

    def _handle_ratio_changed(self, ratio: str) -> None:
        if self.controller is not None and not self._can_change_ratio(self.controller.status):
            self._select_ratio_for_controller(self.controller)
            return
        self._replace_controller(self._build_controller_for_ratio(ratio))

    def _show_message(self, message: str) -> None:
        self.statusbar.showMessage(message)

    def launch_calibrate(self) -> None:
        if self.controller is None:
            return
        if self.calibrate_window is None:
            self.calibrate_window = CalibrateWindow(self.controller)
        self.calibrate_window.show()
        self.calibrate_window.raise_()
        self.calibrate_window.activateWindow()

    def _start_run(self) -> None:
        if self.controller is None:
            return
        self.controller.run_transfer_async()

    def _update_pose(self, pose: Pose) -> None:
        pose_text = f"Pose x={pose.point[0]:.3f} m, y={pose.point[1]:.3f} m, z={pose.point[2]:.3f} m"
        self.poseValueLabel.setText(pose_text)
        self.statusbar.showMessage(pose_text)

    def _update_clusters(self, clusters) -> None:
        self.detectedValueLabel.setText(str(len(clusters)))

    def _update_status(self, status: ControllerStatus) -> None:
        if self.controller is None:
            return

        readiness_text, readiness_color = self._readiness(status)
        self._set_status_badge(self.readinessLabel, readiness_text, readiness_color)

        self.stateLabel.setText(status.state.capitalize() if status.state else "Idle")
        self.detailLabel.setText(status.detail)
        self.statusbar.showMessage(status.detail)

        self.ratioComboBox.setEnabled(self._can_change_ratio(status))
        self.calibrateButton.setEnabled(not status.busy)
        self.runButton.setEnabled(status.calibrated and not status.busy)

        nominal = max(self.controller.config.nominal_clusters, 0)
        outgoing = max(self.controller.config.outgoing_clusters, 0)
        source_offset = self._clamp(status.source_offset, 0, nominal)
        destination_offset = self._clamp(status.destination_offset, 0, outgoing)
        next_batch = min(nominal - source_offset, outgoing - destination_offset)
        next_batch = max(next_batch, 0)

        self.sourceProgressBar.setMaximum(max(nominal, 1))
        self.sourceProgressBar.setValue(source_offset)
        self.sourceCountLabel.setText(f"{source_offset} / {nominal}")

        self.destinationProgressBar.setMaximum(max(outgoing, 1))
        self.destinationProgressBar.setValue(destination_offset)
        self.destinationCountLabel.setText(f"{destination_offset} / {outgoing}")

        self.detectedValueLabel.setText(str(status.detected_count))
        self.transferredValueLabel.setText(str(status.transferred_count))
        self.nextBatchValueLabel.setText(str(next_batch))

    def _readiness(self, status: ControllerStatus) -> tuple[str, str]:
        if status.state == "error":
            return "Error", self.STATUS_COLORS["error"]
        if status.calibrated:
            return "Ready", self.STATUS_COLORS["calibrated"]
        return "Not Ready", self.STATUS_COLORS.get(status.state, self.STATUS_COLORS["idle"])

    @staticmethod
    def _can_change_ratio(status: ControllerStatus) -> bool:
        return status.state == "idle" and not status.busy and not status.calibrated

    @staticmethod
    def _clamp(value: int, minimum: int, maximum: int) -> int:
        return max(minimum, min(value, maximum))

    def _set_status_badge(self, label, text: str, color: str) -> None:
        label.setText(text)
        label.setStyleSheet(
            "QLabel {"
            f"background-color: {color};"
            "color: white;"
            "border-radius: 14px;"
            "font-weight: 700;"
            "padding: 5px 10px;"
            "}"
        )


def main() -> int:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
