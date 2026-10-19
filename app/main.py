
import sys
import math

from PySide6.QtWidgets import (
    QApplication, QGraphicsView, QGraphicsScene
)
from PySide6.QtGui import QPen, QBrush, QColor
from PySide6.QtCore import Qt

from PySide6.QtWidgets import (
    QGraphicsEllipseItem,
    QGraphicsItem,    
    QGraphicsLineItem,
    QGraphicsTextItem
)
from PySide6.QtGui import QPen, QBrush, QColor


class Waypoint(QGraphicsEllipseItem):

    def __init__(self, waypoint_id, x, y, radius=5, label=None):
        super().__init__(
            -radius, -radius,
            radius * 2, radius * 2
        )

        self.waypoint_id = waypoint_id
        self.label = label or str(waypoint_id)
        self.radius = radius

        self.connected_segments = []

        self.setPos(x, y)

        self.setPen(QPen(QColor("blue")))
        self.setBrush(QBrush(QColor("blue")))

        self.setFlag(
            QGraphicsItem.GraphicsItemFlag.ItemIsMovable,
            True
        )

        self.setFlag(
            QGraphicsItem.GraphicsItemFlag.ItemIsSelectable,
            True
        )

        self.setFlag(
            QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges,
            True
        )
    def itemChange(self, change, value):
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:

            for segment in self.connected_segments:
                segment.updateGeometry()

        return super().itemChange(change, value)


class PathSegment(QGraphicsLineItem):
    def __init__(self, start_waypoint, end_waypoint, pixels_per_cm=5):
        super().__init__()
        self.start_waypoint = start_waypoint
        self.end_waypoint = end_waypoint
        self.setPen(QPen(QColor("blue")))
        self.start_waypoint.connected_segments.append(self)
        self.end_waypoint.connected_segments.append(self)
        self.pixels_per_cm=pixels_per_cm
        self.distance_label = QGraphicsTextItem(self)

        self.updateGeometry()
    def updateGeometry(self):
        start_pos = self.start_waypoint.scenePos()
        end_pos = self.end_waypoint.scenePos()

        self.setLine(start_pos.x(), start_pos.y(), end_pos.x(), end_pos.y())
        self.calculateDistance()
    def calculateDistance(self):
            start_pos = self.start_waypoint.scenePos()
            end_pos = self.end_waypoint.scenePos()
            distance_pixels = math.hypot(start_pos.x() - end_pos.x(), start_pos.y() - end_pos.y())
            distance_cm = (
                distance_pixels / self.pixels_per_cm
            )

            self.distance_label.setPlainText(
                f"{distance_cm:.1f} cm"
            )
            self.distance_label.setPos(
                (start_pos.x() + end_pos.x()) / 2,
                (start_pos.y() + end_pos.y()) / 2 - 20
            )

            print(f"Move {distance_cm:.1f} cm")



class RobotCanvas(QGraphicsView):
    def __init__(self):
        super().__init__()

        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setWindowTitle("Robot Path Designer")
        self.resize(900, 650)
        self.next_waypoint_id = 0
        self.waypoints = []

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
        waypoint = Waypoint(self.next_waypoint_id, x, y)
        self.next_waypoint_id += 1
        self.waypoints.append(waypoint)
        a = self.scene.addItem(waypoint)
        print(len(self.waypoints))
        if len(self.waypoints) > 1:
            pathsegment = PathSegment(self.waypoints[-2], self.waypoints[-1])
            b= self.scene.addItem(pathsegment)


app = QApplication(sys.argv)
window = RobotCanvas()
window.show()
sys.exit(app.exec())
