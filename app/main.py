
import sys
import math

from PySide6.QtWidgets import (
    QApplication,
    QGraphicsView,
    QGraphicsScene,
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsLineItem,
    QGraphicsTextItem,
    QFileDialog,
    QGraphicsPixmapItem,
    QToolBar,
    QMainWindow,
    QDockWidget,
    QWidget, 
    QVBoxLayout, 
    QLabel
)
from PySide6.QtPdf import QPdfDocument
from PySide6.QtGui import QPen, QBrush, QColor, QPainter, QPixmap, QAction, QActionGroup
from PySide6.QtCore import Qt, QSize, Signal


class Waypoint(QGraphicsEllipseItem):
    def __init__(self, waypoint_id, x, y, radius=5, label=None):
        super().__init__(-radius, -radius, radius * 2, radius * 2)

        self.waypoint_id = waypoint_id
        self.label = label if label is not None else str(waypoint_id)
        self.radius = radius
        self.connected_segments = []

        self.setPen(QPen(QColor("blue")))
        self.setBrush(QBrush(QColor("blue")))
        self.setZValue(3)

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
            if self.scene() is not None:
                self.scene().parent().geometryChanged.emit(segment)

        return super().itemChange(change, value)


class PathSegment(QGraphicsLineItem):
    def __init__(self, start_waypoint, end_waypoint, pixels_per_cm):
        super().__init__()

        self.start_waypoint = start_waypoint
        self.end_waypoint = end_waypoint
        self.pixels_per_cm = pixels_per_cm

        self.setPen(QPen(QColor("blue"), 2))
        self.setZValue(1)

        self.distance_label = QGraphicsTextItem(self)
        self.distance_label.setDefaultTextColor(QColor("red"))
        self.distance_label.setZValue(2)

        self.start_waypoint.connected_segments.append(self)
        self.end_waypoint.connected_segments.append(self)

        self.updateGeometry()
        self.setFlag(
            QGraphicsItem.GraphicsItemFlag.ItemIsSelectable,
            True
        )


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
        
        self.distance = distance_cm
        
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
    itemSelected = Signal(object)
    geometryChanged = Signal(object)
    def __init__(self):
        super().__init__()
        
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)

        self.setWindowTitle("Robot Path Designer")
        self.resize(1600, 774)

        self.next_waypoint_id = 0
        self.waypoints = []
        self.segments = []

        self.pixels_per_cm = 0.67739204064

        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.scene.setSceneRect(0, 0, 1600, 774)

        self.zoom_level = 1.0
        self.min_zoom = 0.25
        self.max_zoom = 4.0

        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.current_tool = "select"
        self.background = None
        self.scene.selectionChanged.connect(self.onSelectionChanged)
        self.geometryChanged.connect(self.onSelectionChanged)
    def onSelectionChanged(self):
        selected_items = self.scene.selectedItems()

        if not selected_items:
            self.itemSelected.emit(None)

        for item in selected_items:
            if isinstance(item, Waypoint):
                self.itemSelected.emit(item)
                print("Selected waypoint:", item.waypoint_id)

            elif isinstance(item, PathSegment):
                self.itemSelected.emit(item)
                print(f"{item.start_waypoint, item.end_waypoint}")
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
        elif event.key() == Qt.Key.Key_I:
            self.importField()

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
        if self.current_tool == "select":
            return super().mousePressEvent(event)
        if isinstance(item, Waypoint):
            return super().mousePressEvent(event)
        if isinstance(item, PathSegment):
            return super().mousePressEvent(event)

        if item is not None and isinstance(item, QGraphicsPixmapItem)==False:
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
    def importField(self):
        file_path, selected_filter = QFileDialog.getOpenFileName(self, "Import WRO Field", "", "PDF Files (*.pdf)")
        if file_path:
            document = QPdfDocument(self)
            error = document.load(file_path)
            print(error)

            if error != QPdfDocument.Error.None_ or document.pageCount() < 1:
                print("Failed to load PDF")
                return

            print(document.pageCount())
            dimensions = document.pagePointSize(0)

            if dimensions.width() <= 0 or dimensions.height() <= 0:
                print("Invalid PDF dimensions")
                return

            render_width = 1600
            render_height = round(render_width * dimensions.height() / dimensions.width())
            image_size = QSize(render_width, render_height)
            image = document.render(0, image_size)

            if image.isNull():
                print("Failed to render PDF")
                return

            pixmap = QPixmap.fromImage(image)
            item = QGraphicsPixmapItem(pixmap)
            item.setZValue(0)

            if self.background is not None:
                self.scene.removeItem(self.background)

            self.background = item
            self.scene.addItem(item)
            self.scene.setSceneRect(self.scene.itemsBoundingRect())

            self.fitInView(item, Qt.AspectRatioMode.KeepAspectRatio)
            self.zoom_level = self.transform().m11()
        else:
            print("No file selected")

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
    def setTool(self, tool):
        self.current_tool = tool
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Robot Path Designer")
        self.resize(1600, 774)

        self.canvas = RobotCanvas()
        self.canvas.itemSelected.connect(self.updateProperties)

        self.setCentralWidget(self.canvas)

        properties_widget = QWidget()
        properties_layout = QVBoxLayout()
        self.properties_dock = QDockWidget("Properties", self)
        properties_widget.setLayout(properties_layout)
        self.selected_label = QLabel("Selected Item: None")
        properties_layout.addWidget(self.selected_label)
        self.distance_label = QLabel("Distance: ")
        properties_layout.addWidget(self.distance_label)
        self.movement_label = QLabel("Movement Type: ")
        properties_layout.addWidget(self.movement_label)
        self.properties_dock.setWidget(properties_widget)
        properties_layout.addStretch()
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.properties_dock)

        file_menu = self.menuBar().addMenu("File")

        import_action = QAction("Import Field", self)
        import_action.triggered.connect(self.canvas.importField)
        file_menu.addAction(import_action)

        toolbar = QToolBar("Main Toolbar", self)
        self.addToolBar(toolbar)

        toolbar.addAction(import_action)

        tool_group = QActionGroup(self)
        tool_group.setExclusive(True)

        select_action = QAction("Select", self)
        select_action.setCheckable(True)
        select_action.setChecked(True)
        select_action.triggered.connect(lambda: self.canvas.setTool("select"))

        waypoint_action = QAction("Add Waypoint", self)
        waypoint_action.setCheckable(True)
        waypoint_action.triggered.connect(lambda: self.canvas.setTool("waypoint"))

        tool_group.addAction(select_action)
        tool_group.addAction(waypoint_action)

        toolbar.addAction(select_action)
        toolbar.addAction(waypoint_action)

    def updateProperties(self, item):
        if isinstance(item, Waypoint):
            id = item.waypoint_id
            self.selected_label.setText(f"Selected Item: Waypoint {id}")
            self.distance_label.setText("Distance: —")
            self.movement_label.setText(f"Position Coordinates: X: {item.scenePos().x():.1f}, Y: {item.scenePos().y():.1f}")
        if isinstance(item, PathSegment):
            self.selected_label.setText(f"Selected Item: Path Segment connected from {item.start_waypoint.waypoint_id} to {item.end_waypoint.waypoint_id}")
            self.distance_label.setText(f"Distance: {item.distance:.1f} cm")
            self.movement_label.setText("Movement Type: —")
        if item is None:
            self.selected_label.setText("Selected Item: None")
            self.distance_label.setText("Distance: —")
            self.movement_label.setText("Movement Type: —")




if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
