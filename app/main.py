
import sys
import math

from PySide6.QtWidgets import (
    QApplication,
    QGraphicsView,
    QGraphicsScene,
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsLineItem,
    QGraphicsTextItem
)
from PySide6.QtGui import QPen, QBrush, QColor, QPainter
from PySide6.QtCore import Qt


class Waypoint(QGraphicsEllipseItem):
    def __init__(self, waypoint_id, x, y, radius=5, label=None):
        super().__init__(-radius, -radius, radius * 2, radius * 2)

        self.waypoint_id = waypoint_id
        self.label = label if label is not None else str(waypoint_id)
        self.radius = radius
        self.connected_segments = []

        self.setPen(QPen(QColor("blue")))
        self.setBrush(QBrush(QColor("blue")))
        self.setZValue(2)

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

        self.setPos(x, y)

    def itemChange(self, change, value):
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            for segment in self.connected_segments:
                segment.updateGeometry()

        return super().itemChange(change, value)


class PathSegment(QGraphicsLineItem):
    def __init__(self, start_waypoint, end_waypoint, pixels_per_cm):
        super().__init__()

        self.start_waypoint = start_waypoint
        self.end_waypoint = end_waypoint
        self.pixels_per_cm = pixels_per_cm

        self.setPen(QPen(QColor("blue"), 2))
        self.setZValue(0)

        self.distance_label = QGraphicsTextItem(self)
        self.distance_label.setDefaultTextColor(QColor("red"))
        self.distance_label.setZValue(1)

        self.start_waypoint.connected_segments.append(self)
        self.end_waypoint.connected_segments.append(self)

        self.updateGeometry()

    def updateGeometry(self):
        start_pos = self.start_waypoint.scenePos()
        end_pos = self.end_waypoint.scenePos()

        self.setLine(
            start_pos.x(),
            start_pos.y(),
            end_pos.x(),
            end_pos.y()
        )

        self.calculateDistance()

    def calculateDistance(self):
        start_pos = self.start_waypoint.scenePos()
        end_pos = self.end_waypoint.scenePos()

        distance_pixels = math.hypot(
            end_pos.x() - start_pos.x(),
            end_pos.y() - start_pos.y()
        )

        distance_cm = distance_pixels / self.pixels_per_cm

        self.distance_label.setPlainText(f"{distance_cm:.1f} cm")

        midpoint_x = (start_pos.x() + end_pos.x()) / 2
        midpoint_y = (start_pos.y() + end_pos.y()) / 2

        label_width = self.distance_label.boundingRect().width()

        self.distance_label.setPos(
            midpoint_x - label_width / 2,
            midpoint_y - 25
        )

    def disconnect(self):
        self.start_waypoint.connected_segments.remove(self)
        self.end_waypoint.connected_segments.remove(self)


class RobotCanvas(QGraphicsView):
    def __init__(self):
        super().__init__()

        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)

        self.setWindowTitle("Robot Path Designer")
        self.resize(900, 650)

        self.next_waypoint_id = 0
        self.waypoints = []
        self.segments = []

        self.pixels_per_cm = 5

        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.scene.setSceneRect(0, 0, 850, 600)

        self.zoom_level = 1.0
        self.min_zoom = 0.25
        self.max_zoom = 4.0

        self.setTransformationAnchor(
            QGraphicsView.ViewportAnchor.AnchorUnderMouse
        )

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key.Key_Delete, Qt.Key.Key_Backspace):
            selected_items = self.scene.selectedItems()

            for item in selected_items:
                if isinstance(item, Waypoint):
                    copy_of_connected_segments = list(item.connected_segments)
                    for segment in copy_of_connected_segments:
                        self.removeSegment(segment)
                    self.deleteWaypoint(item)
        elif event.key() == Qt.Key.Key_Plus:
            zoom_factor = 1.1

            new_zoom = self.zoom_level * zoom_factor

            if self.min_zoom <= new_zoom <= self.max_zoom:
                self.scale(zoom_factor, zoom_factor)
                self.zoom_level = new_zoom

            event.accept()
        elif event.key() == Qt.Key.Key_Minus:
            zoom_factor = 1/1.1

            new_zoom = self.zoom_level * zoom_factor

            if self.min_zoom <= new_zoom <= self.max_zoom:
                self.scale(zoom_factor, zoom_factor)
                self.zoom_level = new_zoom

            event.accept()

        else:
            super().keyPressEvent(event)
    """ Wheel Scrolling commented out"""
    # def wheelEvent(self, event):
    #     delta = event.angleDelta().y()

    #     if delta == 0:
    #         return

    #     zoom_factor = 1.1 if delta > 0 else 1 / 1.1

    #     new_zoom = self.zoom_level * zoom_factor

    #     if self.min_zoom <= new_zoom <= self.max_zoom:
    #         self.scale(zoom_factor, zoom_factor)
    #         self.zoom_level = new_zoom

    #     event.accept()

    def mousePressEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton:
            return super().mousePressEvent(event)

        viewport_pos = event.position().toPoint()
        item = self.itemAt(viewport_pos)

        if isinstance(item, Waypoint):
            return super().mousePressEvent(event)

        if item is not None:
            return super().mousePressEvent(event)

        scene_pos = self.mapToScene(viewport_pos)
        x, y = scene_pos.x(), scene_pos.y()

        waypoint = Waypoint(
            waypoint_id=self.next_waypoint_id,
            x=x,
            y=y
        )

        self.next_waypoint_id += 1
        self.waypoints.append(waypoint)
        self.scene.addItem(waypoint)

        if len(self.waypoints) > 1:
            pathsegment = PathSegment(
                self.waypoints[-2],
                self.waypoints[-1],
                self.pixels_per_cm
            )

            self.segments.append(pathsegment)
            self.scene.addItem(pathsegment)

    def deleteWaypoint(self, item):
        index = self.waypoints.index(item)
        self.waypoints.remove(item)
        self.scene.removeItem(item)
        self.reconnectSegments(index)

    def removeSegment(self, segment):
        segment.disconnect()
        self.segments.remove(segment)
        self.scene.removeItem(segment)

    def reconnectSegments(self, index):
        if len(self.waypoints) > 1:
            print(len(self.waypoints))
            if index != 0 and index != (len(self.waypoints)):
                pathsegment = PathSegment(self.waypoints[index-1], self.waypoints[index], self.pixels_per_cm)
                self.segments.insert(index-1, pathsegment)
                self.scene.addItem(pathsegment)
                print(self.segments)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotCanvas()
    window.show()
    sys.exit(app.exec())
