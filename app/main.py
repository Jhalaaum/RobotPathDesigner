
import sys
import math

from PySide6.QtWidgets import (
    QApplication, QGraphicsView, QGraphicsScene
)
from PySide6.QtGui import QPen, QBrush, QColor
from PySide6.QtCore import Qt


class RobotCanvas(QGraphicsView):
    def __init__(self):
        super().__init__()

        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setWindowTitle("Robot Path Designer")
        self.resize(900, 650)

        # 5 screen/scene units = 1 cm
        self.pixels_per_cm = 5

        self.points = []

        self.setRenderHint(
            self.renderHints().__class__.Antialiasing
        )

        self.scene.setSceneRect(0, 0, 850, 600)

    def mousePressEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton:
            return super().mousePressEvent(event)

        pos = self.mapToScene(event.position().toPoint())
        x, y = pos.x(), pos.y()

        self.scene.addEllipse(
            x - 4, y - 4, 8, 8,
            QPen(QColor("blue")),
            QBrush(QColor("blue"))
        )

        if self.points:
            x1, y1 = self.points[-1]

            self.scene.addLine(
                x1, y1, x, y,
                QPen(QColor("green"), 2)
            )

            distance_pixels = math.hypot(
                x - x1, y - y1
            )
            distance_cm = (
                distance_pixels / self.pixels_per_cm
            )

            label = self.scene.addText(
                f"{distance_cm:.1f} cm"
            )
            label.setPos(
                (x1 + x) / 2,
                (y1 + y) / 2 - 20
            )

            print(f"Move {distance_cm:.1f} cm")

        self.points.append((x, y))


app = QApplication(sys.argv)
window = RobotCanvas()
window.show()
sys.exit(app.exec())
