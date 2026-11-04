
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
    QLabel,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QGraphicsRectItem
)

from PySide6.QtPdf import QPdfDocument
from PySide6.QtGui import QPen, QBrush, QColor, QPainter, QPixmap, QAction, QActionGroup
from PySide6.QtCore import Qt, QSize, Signal, QSignalBlocker

class Robot:
    def __init__(self, width, length, wheel_diameter, axle_track, initial_heading=0):
        self.width = width
        self.length = length
        self.wheel_diameter = wheel_diameter
        self.axle_track = axle_track
        self.initial_heading = initial_heading

class RobotRectangle(QGraphicsRectItem):
    def __init__(self, robot, pixels_per_mm):
        super().__init__()
        width_scene = robot.width * pixels_per_mm
        length_scene = robot.length * pixels_per_mm

        self.setRect(-width_scene / 2, -length_scene / 2, width_scene, length_scene)
        self.setPen(QPen(QColor("darkgreen"), 2))
        self.setBrush(QBrush(QColor(0, 180, 0, 70)))
        self.setZValue(4)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)


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

        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges, True)

        self.setPos(x, y)

    def itemChange(self, change, value):
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            for segment in self.connected_segments:
                segment.updateGeometry()
            if self.scene() is not None:
                self.scene().parent().geometryChanged.emit(self)

        return super().itemChange(change, value)


class PathSegment(QGraphicsLineItem):
    def __init__(self, start_waypoint, end_waypoint, pixels_per_mm):
        super().__init__()

        self.start_waypoint = start_waypoint
        self.end_waypoint = end_waypoint
        self.pixels_per_mm = pixels_per_mm

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
        self.calculateHeading()

    def calculateDistance(self):
        start_pos = self.start_waypoint.scenePos()
        end_pos = self.end_waypoint.scenePos()

        distance_pixels = math.hypot(
            end_pos.x() - start_pos.x(),
            end_pos.y() - start_pos.y()
        )

        distance_cm = distance_pixels / self.pixels_per_mm / 10
        
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
    def calculateHeading(self):
        start = self.start_waypoint.scenePos()
        end = self.end_waypoint.scenePos()
        dy = end.y() - start.y()
        dx = end.x() - start.x()
        if dx == 0 and dy == 0:
            self.angle_degrees = None
            return

        angle_radians = math.atan2(dy, dx)
        self.angle_degrees = math.degrees(angle_radians)

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

        self.pixels_per_mm = 0.67739204064
        self.robot = Robot(width=250, length=250, wheel_diameter=56, axle_track=120)
        self.robot_item = RobotRectangle(self.robot, self.pixels_per_mm)
        self.scene.addItem(self.robot_item)
        self.robot_item.setPos(400, 300)
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
                # print("Selected waypoint:", item.waypoint_id)

            elif isinstance(item, PathSegment):
                self.itemSelected.emit(item)
                # print(f"{item.start_waypoint, item.end_waypoint}")
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
                self.pixels_per_mm
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
                pathsegment = PathSegment(self.waypoints[index-1], self.waypoints[index], self.pixels_per_mm)
                self.segments.insert(index-1, pathsegment)
                self.scene.addItem(pathsegment)
                print(self.segments)
    def setTool(self, tool):
        self.current_tool = tool
    def calculateTurn(self, waypoint):
        if waypoint not in self.waypoints or waypoint == self.waypoints[0] or waypoint == self.waypoints[-1]:
            return None
        previous_segment = next((segment for segment in waypoint.connected_segments if segment.end_waypoint is waypoint), None)
        next_segment = next((segment for segment in waypoint.connected_segments if segment.start_waypoint is waypoint), None)
        if previous_segment is None or next_segment is None:
            return None
        if previous_segment.angle_degrees is None or next_segment.angle_degrees is None:
            return None
        return (next_segment.angle_degrees - previous_segment.angle_degrees + 180) % 360 - 180
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
        self.selected_label.setWordWrap(True)
        properties_layout.addWidget(self.selected_label)
        self.distance_label = QLabel("Distance: ")
        properties_layout.addWidget(self.distance_label)
        self.angle_label = QLabel("Angle: ")
        properties_layout.addWidget(self.angle_label)
        self.turn_label = QLabel("Turn: —")
        properties_layout.addWidget(self.turn_label)
        self.movement_label = QLabel("Movement Type: ")
        properties_layout.addWidget(self.movement_label)

        position_group = QGroupBox("Waypoint Position")
        position_layout = QFormLayout()
        self.x_input = QDoubleSpinBox()
        self.x_input.setRange(-1000000, 1000000)
        self.x_input.setDecimals(2)
        self.x_input.setSuffix(" scene units")
        self.x_input.setEnabled(False)
        self.y_input = QDoubleSpinBox()
        self.y_input.setRange(-1000000, 1000000)
        self.y_input.setDecimals(2)
        self.y_input.setSuffix(" scene units")
        self.y_input.setEnabled(False)
        position_layout.addRow("X:", self.x_input)
        position_layout.addRow("Y:", self.y_input)
        position_group.setLayout(position_layout)
        properties_layout.addWidget(position_group)
        self.x_input.valueChanged.connect(self.updateWaypointPosition)
        self.y_input.valueChanged.connect(self.updateWaypointPosition)

        self.properties_dock.setWidget(properties_widget)
        self.properties_dock.setMinimumWidth(280)
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
            self.angle_label.setText("Heading: —")
            turn = self.canvas.calculateTurn(item)
            self.turn_label.setText(f"Turn: {turn:.1f}°" if turn is not None else "Turn: —")
            self.movement_label.setText(f"Position Coordinates: X: {item.scenePos().x():.1f}, Y: {item.scenePos().y():.1f}")
            with QSignalBlocker(self.x_input), QSignalBlocker(self.y_input):
                self.x_input.setValue(item.scenePos().x())
                self.y_input.setValue(item.scenePos().y())
            self.x_input.setEnabled(True)
            self.y_input.setEnabled(True)
        if isinstance(item, PathSegment):
            self.selected_label.setText(f"Selected Item: Path Segment connected from {item.start_waypoint.waypoint_id} to {item.end_waypoint.waypoint_id}")
            self.distance_label.setText(f"Distance: {item.distance:.1f} cm")
            self.angle_label.setText(f"Heading: {item.angle_degrees:.1f}°" if item.angle_degrees is not None else "Heading: Undefined")
            self.turn_label.setText("Turn: —")
            self.movement_label.setText("Movement Type: —")
        if item is None:
            self.selected_label.setText("Selected Item: None")
            self.distance_label.setText("Distance: —")
            self.angle_label.setText("Heading: —")
            self.turn_label.setText("Turn: —")
            self.movement_label.setText("Movement Type: —")
        if not isinstance(item, Waypoint):
            self.x_input.setEnabled(False)
            self.y_input.setEnabled(False)

    def updateWaypointPosition(self):
        selected_items = self.canvas.scene.selectedItems()
        if len(selected_items) == 1 and isinstance(selected_items[0], Waypoint):
            selected_items[0].setPos(self.x_input.value(), self.y_input.value())




if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
