
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
    QGraphicsPolygonItem,
    QPushButton,
    QSpinBox,
    QComboBox,
    QScrollArea
)

from PySide6.QtPdf import QPdfDocument
from PySide6.QtGui import QPen, QBrush, QColor, QPainter, QPixmap, QAction, QActionGroup, QPolygonF, QImage
from PySide6.QtCore import Qt, QSize, Signal, QSignalBlocker, QPointF, QTimer, QElapsedTimer

class Robot:
    def __init__(self, width, length, wheel_diameter, axle_track, initial_heading=0):
        self.width = width
        self.length = length
        self.wheel_diameter = wheel_diameter
        self.axle_track = axle_track
        self.initial_heading = initial_heading
        self.sensor_x_mm = 80.0
        self.sensor_y_mm = 0.0
        self.sensor_diameter_mm = 18.0

class RobotLightSensor(QGraphicsEllipseItem):
    def __init__(self, robot_item):
        diameter_scene = robot_item.robot.sensor_diameter_mm * robot_item.pixels_per_mm
        super().__init__(-diameter_scene / 2, -diameter_scene / 2, diameter_scene, diameter_scene, robot_item)
        self.robot_item = robot_item
        self.syncing = False
        self.setPen(QPen(QColor("black"), 2))
        self.setBrush(QBrush(QColor("yellow")))
        self.setZValue(6)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges, True)

    def setSensorPos(self, position):
        self.syncing = True
        self.setPos(position)
        self.syncing = False

    def mousePressEvent(self, event):
        self.robot_item.setSelected(True)
        event.accept()

    def mouseMoveEvent(self, event):
        self.setPos(self.robot_item.mapFromScene(event.scenePos()))
        event.accept()

    def mouseReleaseEvent(self, event):
        event.accept()

    def itemChange(self, change, value):
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged and not self.syncing:
            self.robot_item.robot.sensor_x_mm = value.x() / self.robot_item.pixels_per_mm
            self.robot_item.robot.sensor_y_mm = value.y() / self.robot_item.pixels_per_mm
            if self.robot_item.scene() is not None:
                self.robot_item.scene().parent().geometryChanged.emit(self.robot_item)
        return super().itemChange(change, value)

class RobotVertexHandle(QGraphicsEllipseItem):
    def __init__(self, robot_item, index):
        super().__init__(-6, -6, 12, 12, robot_item)
        self.robot_item = robot_item
        self.index = index
        self.syncing = False
        self.setPen(QPen(QColor("darkgreen"), 2))
        self.setBrush(QBrush(QColor("white")))
        self.setZValue(5)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges, True)
        self.setAcceptedMouseButtons(Qt.MouseButton.LeftButton)
        self.setVisible(False)

    def setHandlePos(self, point):
        self.syncing = True
        self.setPos(point)
        self.syncing = False

    def mousePressEvent(self, event):
        self.robot_item.setSelected(True)
        self.robot_item.selected_vertex = self.index
        self.robot_item.updateHandleStyles()
        if self.robot_item.scene() is not None:
            self.robot_item.scene().parent().geometryChanged.emit(self.robot_item)
        event.accept()

    def mouseMoveEvent(self, event):
        self.setPos(self.robot_item.mapFromScene(event.scenePos()))
        event.accept()

    def mouseReleaseEvent(self, event):
        event.accept()

    def itemChange(self, change, value):
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged and not self.syncing:
            self.robot_item.moveVertex(self.index, value)
        return super().itemChange(change, value)

class RobotPolygon(QGraphicsPolygonItem):
    def __init__(self, robot, pixels_per_mm):
        super().__init__()
        self.robot = robot
        self.pixels_per_mm = pixels_per_mm
        self.edit_mode = False
        self.selected_vertex = None
        self.vertex_handles = []
        width_scene = robot.width * pixels_per_mm
        length_scene = robot.length * pixels_per_mm
        self.vertices = [QPointF(-length_scene / 2, -width_scene / 2), QPointF(length_scene / 2, -width_scene / 2), QPointF(length_scene / 2, width_scene / 2), QPointF(-length_scene / 2, width_scene / 2)]

        self.setPolygon(QPolygonF(self.vertices))
        self.setPen(QPen(QColor("darkgreen"), 2))
        self.setBrush(QBrush(QColor(0, 180, 0, 70)))
        self.setZValue(4)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges, True)

        self.front_line = QGraphicsLineItem(self)
        self.front_left = QGraphicsLineItem(self)
        self.front_right = QGraphicsLineItem(self)
        for line in (self.front_line, self.front_left, self.front_right):
            line.setPen(QPen(QColor("darkorange"), 3))
            line.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self.updateFrontIndicator()
        self.rebuildHandles()
        self.light_sensor = RobotLightSensor(self)
        self.updateSensorPosition()

    def updateSensorPosition(self):
        diameter_scene = self.robot.sensor_diameter_mm * self.pixels_per_mm
        self.light_sensor.setRect(-diameter_scene / 2, -diameter_scene / 2, diameter_scene, diameter_scene)
        self.light_sensor.setSensorPos(QPointF(self.robot.sensor_x_mm * self.pixels_per_mm, self.robot.sensor_y_mm * self.pixels_per_mm))
        if self.scene() is not None:
            self.scene().parent().geometryChanged.emit(self)

    def updateFrontIndicator(self):
        front_x = self.polygon().boundingRect().right() - 12
        self.front_line.setLine(0, 0, front_x, 0)
        self.front_left.setLine(front_x - 10, -8, front_x, 0)
        self.front_right.setLine(front_x - 10, 8, front_x, 0)

    def rebuildHandles(self):
        for handle in self.vertex_handles:
            if self.scene() is not None:
                self.scene().removeItem(handle)
            else:
                handle.setParentItem(None)
        self.vertex_handles = []
        for index, point in enumerate(self.vertices):
            handle = RobotVertexHandle(self, index)
            handle.setHandlePos(point)
            handle.setVisible(self.edit_mode)
            self.vertex_handles.append(handle)
        self.updateHandleStyles()

    def updateHandleStyles(self):
        for handle in self.vertex_handles:
            handle.setBrush(QBrush(QColor("orange") if handle.index == self.selected_vertex else QColor("white")))

    def refreshShape(self):
        self.setPolygon(QPolygonF(self.vertices))
        self.updateFrontIndicator()
        for handle, point in zip(self.vertex_handles, self.vertices):
            handle.setHandlePos(point)
        bounds = self.polygon().boundingRect()
        self.robot.width = bounds.height() / self.pixels_per_mm
        self.robot.length = bounds.width() / self.pixels_per_mm
        if self.scene() is not None:
            self.scene().parent().geometryChanged.emit(self)

    def moveVertex(self, index, point):
        self.vertices[index] = QPointF(point)
        self.refreshShape()

    def addVertex(self):
        if not self.edit_mode:
            return
        longest_index = max(range(len(self.vertices)), key=lambda i: (self.vertices[(i + 1) % len(self.vertices)] - self.vertices[i]).manhattanLength())
        start = self.vertices[longest_index]
        end = self.vertices[(longest_index + 1) % len(self.vertices)]
        self.vertices.insert(longest_index + 1, QPointF((start.x() + end.x()) / 2, (start.y() + end.y()) / 2))
        self.selected_vertex = longest_index + 1
        self.rebuildHandles()
        self.refreshShape()

    def removeVertex(self):
        if not self.edit_mode or self.selected_vertex is None or len(self.vertices) <= 3:
            return
        self.vertices.pop(self.selected_vertex)
        self.selected_vertex = None
        self.rebuildHandles()
        self.refreshShape()

    def setEditMode(self, enabled):
        self.edit_mode = enabled
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, True)
        self.selected_vertex = None
        for handle in self.vertex_handles:
            handle.setVisible(enabled)
        self.updateHandleStyles()

    def updateDimensions(self):
        bounds = self.polygon().boundingRect()
        if bounds.width() <= 0 or bounds.height() <= 0:
            return
        x_scale = self.robot.length * self.pixels_per_mm / bounds.width()
        y_scale = self.robot.width * self.pixels_per_mm / bounds.height()
        self.vertices = [QPointF(point.x() * x_scale, point.y() * y_scale) for point in self.vertices]
        self.refreshShape()

    def itemChange(self, change, value):
        if change in (QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged, QGraphicsItem.GraphicsItemChange.ItemRotationHasChanged):
            if self.scene() is not None:
                self.scene().parent().geometryChanged.emit(self)
        return super().itemChange(change, value)


class Waypoint(QGraphicsEllipseItem):
    def __init__(self, waypoint_id, x, y, radius=5, label=None):
        super().__init__(-radius, -radius, radius * 2, radius * 2)

        self.waypoint_id = waypoint_id
        self.label = label if label is not None else str(waypoint_id)
        self.radius = radius
        self.connected_segments = []
        self.intersection_count = 1
        self.intersection_direction = "Follow Route"

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
        self.movement_type = "Drive Straight"
        self.speed_mm_s = 200.0
        self.turn_speed_deg_s = 180.0
        self.line_target = 48.5
        self.kp = 4.0
        self.ki = 0.0
        self.kd = 8.0
        self.line_side = "right"
        self.line_stop = "Distance"

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


    def updateMovementAppearance(self):
        self.setPen(QPen(QColor("darkgreen") if self.movement_type == "Line Follow (PID)" else QColor("blue"), 2))

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
    previewProgress = Signal(int)
    previewFinished = Signal()
    previewStopped = Signal()
    sensorReading = Signal(str)
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
        self.robot_item = RobotPolygon(self.robot, self.pixels_per_mm)
        self.scene.addItem(self.robot_item)
        self.robot_item.setPos(400, 300)
        self.robot_item.setRotation(self.robot.initial_heading)
        self.preview_item = QGraphicsPolygonItem()
        self.preview_item.setPen(QPen(QColor("darkorange"), 2, Qt.PenStyle.DashLine))
        self.preview_item.setBrush(QBrush(QColor(255, 165, 0, 45)))
        self.preview_item.setZValue(3.5)
        self.preview_item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self.preview_front_line = QGraphicsLineItem(self.preview_item)
        self.preview_front_left = QGraphicsLineItem(self.preview_item)
        self.preview_front_right = QGraphicsLineItem(self.preview_item)
        sensor_size = self.robot.sensor_diameter_mm * self.pixels_per_mm
        self.preview_sensor = QGraphicsEllipseItem(-sensor_size / 2, -sensor_size / 2, sensor_size, sensor_size, self.preview_item)
        self.preview_sensor.setPen(QPen(QColor("black"), 2))
        self.preview_sensor.setBrush(QBrush(QColor("yellow")))
        self.preview_sensor.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self.preview_sensor.setZValue(6)
        for line in (self.preview_front_line, self.preview_front_left, self.preview_front_right):
            line.setPen(QPen(QColor("darkorange"), 2))
            line.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self.preview_enabled = False
        self.preview_index = 0
        self.preview_mode = "Departure"
        self.preview_timer = QTimer(self)
        self.preview_timer.setInterval(16)
        self.preview_timer.timeout.connect(self.advanceRobotPreview)
        self.preview_clock = QElapsedTimer()
        self.preview_playing = False
        self.preview_paused = False
        self.preview_segment_index = 0
        self.preview_phase = "turn"
        self.preview_phase_progress = 0.0
        self.preview_heading = self.robot.initial_heading
        self.preview_turn_start = self.robot.initial_heading
        self.preview_turn_target = self.robot.initial_heading
        self.preview_drive_start = QPointF()
        self.preview_speed_mm_s = 200.0
        self.preview_turn_speed_deg_s = 180.0
        self.line_last_error = 0.0
        self.line_integral = 0.0
        self.line_travel_mm = 0.0
        self.line_last_reading = None
        self.line_detected_intersection = False
        self.line_intersections_found = 0
        self.line_intersection_armed = False
        self.line_last_intersection_mm = -1000.0
        self.line_alignment_heading = 0.0
        self.line_align_distance_mm = 0.0
        self.line_approach_phase = False
        self.line_crossing_phase = False
        self.preview_drive_start = QPointF()
        self.syncing_start = False
        self.scene.addItem(self.preview_item)
        self.preview_item.setVisible(False)
        self.geometryChanged.connect(self.syncStartingPose)
        self.geometryChanged.connect(self.refreshRobotPreview)
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.scene.setSceneRect(0, 0, 1600, 774)

        self.zoom_level = 1.0
        self.min_zoom = 0.25
        self.max_zoom = 4.0

        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.current_tool = "select"
        self.background = None
        self.background_image = None
        self.track_debug_enabled = False
        self.track_debug_item = None
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
            elif isinstance(item, RobotPolygon):
                self.itemSelected.emit(item)
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
        if item is self.preview_item or (item is not None and item.parentItem() is self.preview_item):
            item = next((candidate for candidate in self.scene.items(self.mapToScene(viewport_pos)) if candidate is not self.preview_item and candidate.parentItem() is not self.preview_item), None)
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
        if len(self.waypoints) == 1:
            self.robot_item.setPos(waypoint.scenePos())

        if len(self.waypoints) > 1:
            pathsegment = PathSegment(
                self.waypoints[-2],
                self.waypoints[-1],
                self.pixels_per_mm
            )

            self.segments.append(pathsegment)
            self.scene.addItem(pathsegment)
        self.refreshRobotPreview()
        self.geometryChanged.emit(waypoint)
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
            self.background_image = image
            self.track_debug_item = None
            self.scene.addItem(item)
            if self.track_debug_enabled:
                self.updateTrackDebugOverlay()
            self.scene.setSceneRect(self.scene.itemsBoundingRect())

            self.fitInView(item, Qt.AspectRatioMode.KeepAspectRatio)
            self.zoom_level = self.transform().m11()
        else:
            print("No file selected")

    def setTrackDebugVisible(self, enabled):
        self.track_debug_enabled = enabled
        self.updateTrackDebugOverlay()

    def updateTrackDebugOverlay(self):
        if self.background is None or self.background_image is None:
            return
        if not self.track_debug_enabled:
            if self.track_debug_item is not None:
                self.track_debug_item.setVisible(False)
            return
        if self.track_debug_item is None:
            image = self.background_image
            # Half-resolution visualisation for responsive toggling. This is a display only:
            # the robot's sensor still samples the original, full-resolution PDF.
            width = (image.width() + 1) // 2
            height = (image.height() + 1) // 2
            overlay = QImage(width, height, QImage.Format.Format_ARGB32)
            overlay.fill(Qt.GlobalColor.transparent)
            marked = QColor(0, 230, 235, 185).rgba()
            for y in range(height):
                source_y = min(y * 2, image.height() - 1)
                for x in range(width):
                    color = image.pixelColor(min(x * 2, image.width() - 1), source_y)
                    darkest = min(color.red(), color.green(), color.blue())
                    brightest = max(color.red(), color.green(), color.blue())
                    if brightest < 110 and brightest - darkest < 45:
                        overlay.setPixel(x, y, marked)
            self.track_debug_item = QGraphicsPixmapItem(QPixmap.fromImage(overlay), self.background)
            self.track_debug_item.setScale(2.0)
            self.track_debug_item.setZValue(0.5)
            self.track_debug_item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            self.track_debug_item.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, False)
        self.track_debug_item.setVisible(True)

    def deleteWaypoint(self, item):
        index = self.waypoints.index(item)
        self.waypoints.remove(item)
        self.scene.removeItem(item)
        self.reconnectSegments(index)
        self.refreshRobotPreview()
        self.geometryChanged.emit(None)

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
    def syncStartingPose(self, item=None):
        if self.syncing_start:
            return
        if isinstance(item, RobotPolygon):
            self.robot.initial_heading = self.robot_item.rotation()
        if not self.waypoints:
            return
        start = self.waypoints[0]
        if item is not start and item is not self.robot_item and item is not None:
            return
        self.syncing_start = True
        try:
            if item is self.robot_item:
                if start.scenePos() != self.robot_item.scenePos():
                    start.setPos(self.robot_item.scenePos())
            elif start.scenePos() != self.robot_item.scenePos():
                self.robot_item.setPos(start.scenePos())
        finally:
            self.syncing_start = False

    def setRobotPreview(self, enabled, index, mode):
        self.pauseRobotPreview()
        self.preview_paused = False
        self.preview_enabled = enabled
        self.preview_index = index
        self.preview_mode = mode
        self.refreshRobotPreview()

    def refreshRobotPreview(self, item=None):
        if item is not None and self.preview_playing:
            self.pauseRobotPreview()
            self.preview_paused = False
        if not self.preview_enabled or not self.waypoints:
            self.preview_item.setVisible(False)
            return
        self.preview_item.setPolygon(self.robot_item.polygon())
        front_x = self.preview_item.polygon().boundingRect().right() - 12
        self.preview_front_line.setLine(0, 0, front_x, 0)
        self.preview_front_left.setLine(front_x - 10, -8, front_x, 0)
        self.preview_front_right.setLine(front_x - 10, 8, front_x, 0)
        if not self.preview_playing:
            self.preview_index = max(0, min(self.preview_index, len(self.waypoints) - 1))
            waypoint = self.waypoints[self.preview_index]
            incoming = next((segment for segment in waypoint.connected_segments if segment.end_waypoint is waypoint), None)
            outgoing = next((segment for segment in waypoint.connected_segments if segment.start_waypoint is waypoint), None)
            if self.preview_index == 0:
                heading = self.robot.initial_heading
            elif self.preview_mode == "Arrival":
                heading = incoming.angle_degrees if incoming is not None else self.robot.initial_heading
            else:
                heading = outgoing.angle_degrees if outgoing is not None else (incoming.angle_degrees if incoming is not None else self.robot.initial_heading)
            if heading is None:
                self.preview_item.setVisible(False)
                return
            self.preview_item.setPos(waypoint.scenePos())
            self.preview_item.setRotation(heading)
        self.preview_item.setVisible(True)
        self.updatePreviewSensor()

    def updatePreviewSensor(self):
        sensor_size = self.robot.sensor_diameter_mm * self.pixels_per_mm
        self.preview_sensor.setRect(-sensor_size / 2, -sensor_size / 2, sensor_size, sensor_size)
        self.preview_sensor.setPos(self.robot.sensor_x_mm * self.pixels_per_mm, self.robot.sensor_y_mm * self.pixels_per_mm)
        self.sensorReading.emit(self.readLightSensor(self.preview_sensor.scenePos()))

    def sampleTrackReflectance(self, scene_position):
        if self.background is None or self.background_image is None:
            return None
        position = self.background.mapFromScene(scene_position)
        image = self.background_image
        x, y = position.x(), position.y()
        radius = self.robot.sensor_diameter_mm * self.pixels_per_mm / 2
        if x - radius < 0 or y - radius < 0 or x + radius >= image.width() or y + radius >= image.height():
            return None
        black_samples = 0
        sample_count = 0
        for dy in range(-5, 6):
            for dx in range(-5, 6):
                if dx * dx + dy * dy > 25:
                    continue
                color = image.pixelColor(round(x + dx * radius / 5), round(y + dy * radius / 5))
                darkest = min(color.red(), color.green(), color.blue())
                brightest = max(color.red(), color.green(), color.blue())
                if brightest < 110 and brightest - darkest < 45:
                    black_samples += 1
                sample_count += 1
        black_fraction = black_samples / sample_count
        return 94.0 - black_fraction * (94.0 - 3.0)

    def readLightSensor(self, scene_position):
        reading = self.sampleTrackReflectance(scene_position)
        if reading is None:
            return "Light sensor: Import the field PDF and position the sensor on the mat"
        return f"Simulated reflection: {reading:.1f}% (black 3%, white 94%; PDF edge model)"

    def beginPreviewDrive(self):
        self.preview_drive_start = QPointF(self.preview_item.pos())
        self.line_last_error = 0.0
        self.line_integral = 0.0
        self.line_travel_mm = 0.0
        self.line_last_reading = None
        self.line_detected_intersection = False
        self.line_intersections_found = 0
        self.line_intersection_armed = False
        self.line_last_intersection_mm = -1000.0
        self.line_alignment_heading = self.preview_heading
        self.line_align_distance_mm = 0.0
        self.line_approach_phase = False
        self.line_crossing_phase = False

    def advanceLineTrack(self, segment, elapsed):
        distance_mm = segment.distance * 10
        waypoint = segment.end_waypoint
        if distance_mm <= 0:
            self.finishPreviewSegment()
            return
        for _ in range(max(1, math.ceil(elapsed / 0.01))):
            if not self.preview_playing:
                return
            dt = min(0.01, elapsed)
            elapsed -= dt
            self.updatePreviewSensor()
            reading = self.sampleTrackReflectance(self.preview_sensor.scenePos())
            if reading is None:
                self.pauseRobotPreview()
                self.sensorReading.emit("Line tracking stopped: sensor outside imported PDF or no field loaded")
                return
            base_dps = segment.speed_mm_s * 360 / (math.pi * max(1.0, self.robot.wheel_diameter))
            wheel_factor = math.pi * self.robot.wheel_diameter / 360.0
            if self.line_crossing_phase and reading >= 35 and self.line_travel_mm - self.line_last_intersection_mm >= 15:
                self.line_crossing_phase = False
                self.line_intersection_armed = True
                self.line_last_error = segment.line_target - reading
            if self.line_approach_phase or self.line_crossing_phase:
                heading_error = (self.line_alignment_heading - self.preview_heading + 180) % 360 - 180
                correction = max(-base_dps * 0.25, min(base_dps * 0.25, heading_error * 5.0))
                left_dps, right_dps = base_dps + correction, base_dps - correction
            else:
                error = segment.line_target - reading
                derivative = error - self.line_last_error
                self.line_integral = max(-1000.0, min(1000.0, self.line_integral + error * dt))
                steering = error * segment.kp + derivative * segment.kd + self.line_integral * segment.ki
                max_steering = max(45.0, abs(base_dps) * 0.75)
                steering = max(-max_steering, min(max_steering, steering))
                if segment.line_side == "right":
                    left_dps, right_dps = base_dps + steering, base_dps - steering
                else:
                    left_dps, right_dps = base_dps - steering, base_dps + steering
                self.line_last_error = error
                if reading >= 35:
                    self.line_intersection_armed = True
                if segment.line_stop == "Intersection" and self.line_intersection_armed and reading <= 17 and self.line_travel_mm >= 20 and self.line_travel_mm - self.line_last_intersection_mm >= 25:
                    self.line_intersections_found += 1
                    self.line_intersection_armed = False
                    self.line_last_intersection_mm = self.line_travel_mm
                    self.sensorReading.emit(f"Intersection {self.line_intersections_found}/{waypoint.intersection_count} detected")
                    if self.line_intersections_found >= waypoint.intersection_count:
                        self.line_detected_intersection = True
                        self.line_approach_phase = True
                        self.line_alignment_heading = self.preview_heading
                    else:
                        self.line_crossing_phase = True
                        self.line_alignment_heading = self.preview_heading
            left_speed, right_speed = left_dps * wheel_factor, right_dps * wheel_factor
            angular_speed = (left_speed - right_speed) / max(1.0, self.robot.axle_track)
            mid_heading = math.radians(self.preview_heading) + angular_speed * dt / 2
            travel = (left_speed + right_speed) * dt / 2
            self.preview_item.setPos(self.preview_item.pos() + QPointF(math.cos(mid_heading) * travel * self.pixels_per_mm, math.sin(mid_heading) * travel * self.pixels_per_mm))
            self.preview_heading = (self.preview_heading + math.degrees(angular_speed * dt) + 180) % 360 - 180
            self.preview_item.setRotation(self.preview_heading)
            self.line_travel_mm += abs(travel)
            self.line_last_reading = reading
            if self.line_approach_phase:
                target = waypoint.scenePos()
                remaining_x = (target.x() - self.preview_item.pos().x()) / self.pixels_per_mm
                remaining_y = (target.y() - self.preview_item.pos().y()) / self.pixels_per_mm
                direction = math.radians(self.line_alignment_heading)
                forward_remaining = remaining_x * math.cos(direction) + remaining_y * math.sin(direction)
                lateral_error = abs(-remaining_x * math.sin(direction) + remaining_y * math.cos(direction))
                self.line_align_distance_mm += abs(travel)
                if forward_remaining <= 2 and lateral_error <= 30:
                    self.sensorReading.emit(f"Intersection {self.line_intersections_found}: robot centre aligned with waypoint {waypoint.waypoint_id}")
                    self.finishPreviewSegment()
                    return
                if forward_remaining < -10 or self.line_align_distance_mm > max(150, distance_mm * 0.6):
                    self.pauseRobotPreview()
                    self.sensorReading.emit(f"Intersection alignment failed: waypoint {waypoint.waypoint_id} is not on the robot's forward trajectory (lateral error {lateral_error:.0f} mm)")
                    return
            elif segment.line_stop == "Distance" and self.line_travel_mm >= distance_mm:
                self.finishPreviewSegment()
                return
            if self.line_travel_mm >= max(distance_mm * 1.8 + 100, distance_mm + 150):
                self.pauseRobotPreview()
                self.sensorReading.emit(f"Line tracking stopped: detected {self.line_intersections_found}/{waypoint.intersection_count} intersections before distance safety limit")
                return

    def playRobotPreview(self):
        if not self.waypoints or len(self.waypoints) < 2:
            return
        if any(segment.movement_type == "Line Follow (PID)" for segment in self.segments) and self.background_image is None:
            self.sensorReading.emit("Line tracking requires an imported field PDF")
            return
        if not self.preview_enabled:
            self.preview_enabled = True
        if self.preview_playing:
            return
        if not self.preview_paused:
            self.preview_segment_index = 0
            waypoint = self.waypoints[0]
            self.preview_index = 0
            self.previewProgress.emit(0)
            self.preview_heading = self.robot.initial_heading
            self.preview_item.setPos(waypoint.scenePos())
            self.preview_item.setRotation(self.preview_heading)
            self.preview_phase = "turn"
            self.preview_phase_progress = 0.0
            self.preview_turn_start = self.preview_heading
            self.preview_turn_target = self.preview_heading
        self.preview_item.setVisible(True)
        self.updatePreviewSensor()
        self.preview_playing = True
        self.preview_paused = False
        self.preview_clock.start()
        self.preview_timer.start()

    def pauseRobotPreview(self):
        self.preview_paused = self.preview_playing and self.preview_segment_index < len(self.waypoints) - 1
        self.preview_playing = False
        self.preview_timer.stop()
        self.previewStopped.emit()

    def resetRobotPreview(self):
        self.pauseRobotPreview()
        self.preview_paused = False
        self.preview_index = 0
        self.preview_mode = "Departure"
        self.refreshRobotPreview()
        self.previewProgress.emit(0)

    def advanceRobotPreview(self):
        if not self.preview_playing or self.preview_segment_index >= len(self.waypoints) - 1:
            self.pauseRobotPreview()
            return
        remaining = min(self.preview_clock.restart() / 1000.0, 0.1)
        for _ in range(12):
            if remaining <= 0 or not self.preview_playing:
                break
            start = self.waypoints[self.preview_segment_index].scenePos()
            end = self.waypoints[self.preview_segment_index + 1].scenePos()
            dx = end.x() - start.x()
            dy = end.y() - start.y()
            distance = math.hypot(dx, dy)
            if distance == 0:
                self.finishPreviewSegment()
                continue
            target_heading = math.degrees(math.atan2(dy, dx))
            segment = next((segment for segment in self.segments if segment.start_waypoint is self.waypoints[self.preview_segment_index] and segment.end_waypoint is self.waypoints[self.preview_segment_index + 1]), None)
            if self.preview_phase == "turn":
                if self.preview_phase_progress == 0:
                    self.preview_turn_start = self.preview_heading
                    self.preview_turn_target = target_heading
                    origin = self.waypoints[self.preview_segment_index]
                    incoming = next((candidate for candidate in origin.connected_segments if candidate.end_waypoint is origin and candidate.movement_type == "Line Follow (PID)" and candidate.line_stop == "Intersection"), None)
                    if incoming is not None and origin.intersection_direction != "Follow Route":
                        branch_angle = {"Left": -90, "Right": 90, "Straight": 0}[origin.intersection_direction]
                        self.preview_turn_target = (self.preview_heading + branch_angle + 180) % 360 - 180
                delta = (self.preview_turn_target - self.preview_turn_start + 180) % 360 - 180
                duration = abs(delta) / (segment.turn_speed_deg_s if segment is not None else self.preview_turn_speed_deg_s)
                if duration <= 0:
                    self.preview_heading = self.preview_turn_target
                    self.preview_item.setRotation(self.preview_heading)
                    self.preview_phase = "drive"
                    self.preview_phase_progress = 0.0
                    self.beginPreviewDrive()
                    continue
                available = (1.0 - self.preview_phase_progress) * duration
                taken = min(remaining, available)
                self.preview_phase_progress = min(1.0, self.preview_phase_progress + taken / duration)
                self.preview_item.setRotation(self.preview_turn_start + delta * self.preview_phase_progress)
                remaining -= taken
                if self.preview_phase_progress >= 1.0:
                    self.preview_heading = self.preview_turn_target
                    self.preview_phase = "drive"
                    self.preview_phase_progress = 0.0
                    self.beginPreviewDrive()
            else:
                if segment is not None and segment.movement_type == "Line Follow (PID)":
                    self.advanceLineTrack(segment, remaining)
                    remaining = 0
                    continue
                duration = distance / ((segment.speed_mm_s if segment is not None else self.preview_speed_mm_s) * self.pixels_per_mm)
                available = (1.0 - self.preview_phase_progress) * duration
                taken = min(remaining, available)
                self.preview_phase_progress = min(1.0, self.preview_phase_progress + taken / duration)
                direction = math.radians(self.preview_heading)
                self.preview_item.setPos(self.preview_drive_start + QPointF(math.cos(direction) * distance * self.preview_phase_progress, math.sin(direction) * distance * self.preview_phase_progress))
                remaining -= taken
                if self.preview_phase_progress >= 1.0:
                    self.finishPreviewSegment()
            self.updatePreviewSensor()

    def finishPreviewSegment(self):
        self.preview_segment_index += 1
        self.preview_index = self.preview_segment_index
        self.previewProgress.emit(self.preview_index)
        self.preview_phase = "turn"
        self.preview_phase_progress = 0.0
        if self.preview_segment_index >= len(self.waypoints) - 1:
            self.pauseRobotPreview()
            self.preview_paused = False
            self.previewFinished.emit()

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

        self.robot_group = QGroupBox("Robot Properties")
        robot_layout = QFormLayout()
        self.robot_x_input = QDoubleSpinBox()
        self.robot_x_input.setRange(-1000000, 1000000)
        self.robot_x_input.setDecimals(2)
        self.robot_x_input.setSuffix(" scene units")
        self.robot_y_input = QDoubleSpinBox()
        self.robot_y_input.setRange(-1000000, 1000000)
        self.robot_y_input.setDecimals(2)
        self.robot_y_input.setSuffix(" scene units")
        self.robot_heading_input = QDoubleSpinBox()
        self.robot_heading_input.setRange(-360, 360)
        self.robot_heading_input.setDecimals(1)
        self.robot_heading_input.setSuffix("°")
        self.robot_width_input = QDoubleSpinBox()
        self.robot_width_input.setRange(1, 5000)
        self.robot_width_input.setDecimals(1)
        self.robot_width_input.setSuffix(" mm")
        self.robot_length_input = QDoubleSpinBox()
        self.robot_length_input.setRange(1, 5000)
        self.robot_length_input.setDecimals(1)
        self.robot_length_input.setSuffix(" mm")
        robot_layout.addRow("X:", self.robot_x_input)
        robot_layout.addRow("Y:", self.robot_y_input)
        robot_layout.addRow("Heading:", self.robot_heading_input)
        robot_layout.addRow("Width:", self.robot_width_input)
        robot_layout.addRow("Length:", self.robot_length_input)
        self.robot_group.setLayout(robot_layout)
        self.robot_group.setEnabled(False)
        properties_layout.addWidget(self.robot_group)
        self.robot_x_input.valueChanged.connect(self.updateRobotProperties)
        self.robot_y_input.valueChanged.connect(self.updateRobotProperties)
        self.robot_heading_input.valueChanged.connect(self.updateRobotProperties)
        self.robot_width_input.valueChanged.connect(self.updateRobotProperties)
        self.robot_length_input.valueChanged.connect(self.updateRobotProperties)

        self.sensor_group = QGroupBox("Light Sensor Position")
        sensor_layout = QFormLayout()
        self.sensor_x_input = QDoubleSpinBox()
        self.sensor_x_input.setRange(-1000, 1000)
        self.sensor_x_input.setDecimals(1)
        self.sensor_x_input.setSuffix(" mm")
        self.sensor_y_input = QDoubleSpinBox()
        self.sensor_y_input.setRange(-1000, 1000)
        self.sensor_y_input.setDecimals(1)
        self.sensor_y_input.setSuffix(" mm")
        self.sensor_size_input = QDoubleSpinBox()
        self.sensor_size_input.setRange(1, 250)
        self.sensor_size_input.setDecimals(1)
        self.sensor_size_input.setSuffix(" mm")
        self.sensor_size_input.setToolTip("Approximate optical sensing diameter, not the LEGO sensor housing size")
        sensor_layout.addRow("Forward (+X):", self.sensor_x_input)
        sensor_layout.addRow("Right (+Y):", self.sensor_y_input)
        sensor_layout.addRow("Sensing diameter:", self.sensor_size_input)
        self.sensor_group.setLayout(sensor_layout)
        properties_layout.addWidget(self.sensor_group)
        self.sensor_group.setEnabled(False)
        self.sensor_x_input.valueChanged.connect(self.updateSensorProperties)
        self.sensor_y_input.valueChanged.connect(self.updateSensorProperties)
        self.sensor_size_input.valueChanged.connect(self.updateSensorProperties)

        self.intersection_group = QGroupBox("Waypoint Intersection")
        intersection_layout = QFormLayout()
        self.intersection_count_input = QSpinBox()
        self.intersection_count_input.setRange(1, 99)
        self.intersection_direction_input = QComboBox()
        self.intersection_direction_input.addItems(["Follow Route", "Left", "Right", "Straight"])
        intersection_layout.addRow("Stop at intersection #:", self.intersection_count_input)
        intersection_layout.addRow("Turn at waypoint:", self.intersection_direction_input)
        self.intersection_group.setLayout(intersection_layout)
        properties_layout.addWidget(self.intersection_group)
        self.intersection_group.setEnabled(False)
        self.intersection_count_input.valueChanged.connect(self.updateIntersectionProperties)
        self.intersection_direction_input.currentTextChanged.connect(self.updateIntersectionProperties)

        self.movement_group = QGroupBox("Segment Movement")
        self.movement_group.setToolTip("Line Follow uses the imported PDF's black track, a simulated edge sensor, and wheel-speed PD/PID corrections.")
        movement_layout = QFormLayout()
        self.movement_type_input = QComboBox()
        self.movement_type_input.addItems(["Drive Straight", "Line Follow (PID)"])
        self.speed_input = QDoubleSpinBox()
        self.speed_input.setRange(1, 2000)
        self.speed_input.setSuffix(" mm/s")
        self.turn_speed_input = QDoubleSpinBox()
        self.turn_speed_input.setRange(1, 720)
        self.turn_speed_input.setSuffix(" °/s")
        self.line_side_input = QComboBox()
        self.line_side_input.addItems(["right", "left"])
        self.line_side_input.setToolTip("Choose which edge of the black line the sensor follows. Applies to both Distance and Intersection stopping modes.")
        self.line_stop_input = QComboBox()
        self.line_stop_input.addItems(["Distance", "Intersection"])
        self.line_target_input = QDoubleSpinBox()
        self.line_target_input.setRange(0, 100)
        self.line_target_input.setSuffix(" %")
        self.kp_input = QDoubleSpinBox()
        self.ki_input = QDoubleSpinBox()
        self.kd_input = QDoubleSpinBox()
        for control in (self.kp_input, self.ki_input, self.kd_input):
            control.setRange(0, 1000)
            control.setDecimals(3)
            control.setSingleStep(0.1)
        movement_layout.addRow("Movement:", self.movement_type_input)
        movement_layout.addRow("Drive speed:", self.speed_input)
        movement_layout.addRow("Turn speed:", self.turn_speed_input)
        movement_layout.addRow("Line stop:", self.line_stop_input)
        movement_layout.addRow("Track edge (both modes):", self.line_side_input)
        movement_layout.addRow("Line target:", self.line_target_input)
        movement_layout.addRow("Kp:", self.kp_input)
        movement_layout.addRow("Ki:", self.ki_input)
        movement_layout.addRow("Kd:", self.kd_input)
        self.movement_group.setLayout(movement_layout)
        self.movement_group.setEnabled(False)
        properties_layout.addWidget(self.movement_group)
        self.movement_type_input.currentTextChanged.connect(self.updateMovementProperties)
        self.line_side_input.currentTextChanged.connect(self.updateMovementProperties)
        self.line_stop_input.currentTextChanged.connect(self.updateMovementProperties)
        for control in (self.speed_input, self.turn_speed_input, self.line_target_input, self.kp_input, self.ki_input, self.kd_input):
            control.valueChanged.connect(self.updateMovementProperties)

        self.robot_edit_shape_button = QPushButton("Edit Robot Shape")
        self.robot_edit_shape_button.setCheckable(True)
        self.robot_add_vertex_button = QPushButton("Add Vertex")
        self.robot_remove_vertex_button = QPushButton("Remove Selected Vertex")
        self.robot_group.layout().addRow(self.robot_edit_shape_button)
        self.robot_group.layout().addRow(self.robot_add_vertex_button)
        self.robot_group.layout().addRow(self.robot_remove_vertex_button)
        self.robot_edit_shape_button.toggled.connect(self.setRobotShapeEditing)
        self.robot_add_vertex_button.clicked.connect(self.addRobotVertex)
        self.robot_remove_vertex_button.clicked.connect(self.removeRobotVertex)
        self.robot_add_vertex_button.setEnabled(False)
        self.robot_remove_vertex_button.setEnabled(False)

        self.sensor_reading_label = QLabel("Light sensor: —")
        self.sensor_reading_label.setWordWrap(True)
        properties_layout.addWidget(self.sensor_reading_label)
        self.canvas.sensorReading.connect(self.sensor_reading_label.setText)

        self.preview_group = QGroupBox("Robot Path Preview")
        preview_layout = QFormLayout()
        self.preview_button = QPushButton("Show Preview")
        self.preview_button.setCheckable(True)
        self.preview_index_input = QSpinBox()
        self.preview_index_input.setMinimum(0)
        self.preview_index_input.setMaximum(0)
        self.preview_mode_input = QComboBox()
        self.preview_mode_input.addItems(["Departure", "Arrival"])
        self.preview_previous_button = QPushButton("Previous Waypoint")
        self.preview_next_button = QPushButton("Next Waypoint")
        self.preview_play_button = QPushButton("Play Smooth Preview")
        self.preview_pause_button = QPushButton("Pause")
        self.preview_reset_button = QPushButton("Reset to Start")
        self.preview_pause_button.setEnabled(False)
        preview_layout.addRow(self.preview_button)
        preview_layout.addRow("Waypoint Index:", self.preview_index_input)
        preview_layout.addRow("Orientation:", self.preview_mode_input)
        preview_layout.addRow(self.preview_previous_button)
        preview_layout.addRow(self.preview_next_button)
        preview_layout.addRow(self.preview_play_button)
        preview_layout.addRow(self.preview_pause_button)
        preview_layout.addRow(self.preview_reset_button)
        self.preview_group.setLayout(preview_layout)
        properties_layout.addWidget(self.preview_group)
        self.preview_button.toggled.connect(self.updateRobotPreview)
        self.preview_index_input.valueChanged.connect(self.updateRobotPreview)
        self.preview_mode_input.currentTextChanged.connect(self.updateRobotPreview)
        self.preview_previous_button.clicked.connect(lambda: self.preview_index_input.setValue(self.preview_index_input.value() - 1))
        self.preview_next_button.clicked.connect(lambda: self.preview_index_input.setValue(self.preview_index_input.value() + 1))
        self.preview_play_button.clicked.connect(self.playSmoothPreview)
        self.preview_pause_button.clicked.connect(self.pauseSmoothPreview)
        self.preview_reset_button.clicked.connect(self.resetSmoothPreview)
        self.canvas.previewProgress.connect(self.onPreviewProgress)
        self.canvas.previewFinished.connect(self.onPreviewFinished)
        self.canvas.previewStopped.connect(self.onPreviewFinished)
        self.canvas.geometryChanged.connect(self.updatePreviewControls)
        self.updatePreviewControls()

        properties_scroll = QScrollArea()
        properties_scroll.setWidgetResizable(True)
        properties_scroll.setWidget(properties_widget)
        self.properties_dock.setWidget(properties_scroll)
        self.properties_dock.setMinimumWidth(320)
        properties_layout.addStretch()
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.properties_dock)

        file_menu = self.menuBar().addMenu("File")

        import_action = QAction("Import Field", self)
        import_action.triggered.connect(self.canvas.importField)
        file_menu.addAction(import_action)

        toolbar = QToolBar("Main Toolbar", self)
        self.addToolBar(toolbar)

        toolbar.addAction(import_action)

        track_debug_action = QAction("Show Detected Track", self)
        track_debug_action.setCheckable(True)
        track_debug_action.setToolTip("Highlight pixels treated as black by the line tracker (cyan). Read-only; does not affect editing or sensor readings.")
        track_debug_action.toggled.connect(self.canvas.setTrackDebugVisible)
        toolbar.addAction(track_debug_action)
        view_menu = self.menuBar().addMenu("View")
        view_menu.addAction(track_debug_action)

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
            with QSignalBlocker(self.intersection_count_input), QSignalBlocker(self.intersection_direction_input):
                self.intersection_count_input.setValue(item.intersection_count)
                self.intersection_direction_input.setCurrentText(item.intersection_direction)
        if isinstance(item, PathSegment):
            with QSignalBlocker(self.movement_type_input), QSignalBlocker(self.speed_input), QSignalBlocker(self.turn_speed_input), QSignalBlocker(self.line_side_input), QSignalBlocker(self.line_stop_input), QSignalBlocker(self.line_target_input), QSignalBlocker(self.kp_input), QSignalBlocker(self.ki_input), QSignalBlocker(self.kd_input):
                self.movement_type_input.setCurrentText(item.movement_type)
                self.speed_input.setValue(item.speed_mm_s)
                self.turn_speed_input.setValue(item.turn_speed_deg_s)
                self.line_side_input.setCurrentText(item.line_side)
                self.line_stop_input.setCurrentText(item.line_stop)
                self.line_target_input.setValue(item.line_target)
                self.kp_input.setValue(item.kp)
                self.ki_input.setValue(item.ki)
                self.kd_input.setValue(item.kd)
            self.selected_label.setText(f"Selected Item: Path Segment connected from {item.start_waypoint.waypoint_id} to {item.end_waypoint.waypoint_id}")
            self.distance_label.setText(f"Distance: {item.distance:.1f} cm")
            self.angle_label.setText(f"Heading: {item.angle_degrees:.1f}°" if item.angle_degrees is not None else "Heading: Undefined")
            self.turn_label.setText("Turn: —")
            self.movement_label.setText(f"Movement Type: {item.movement_type}")
        if isinstance(item, RobotPolygon):
            self.selected_label.setText("Selected Item: Robot")
            self.distance_label.setText("Distance: —")
            self.angle_label.setText(f"Heading: {item.rotation():.1f}°")
            self.turn_label.setText("Turn: —")
            self.movement_label.setText(f"Position: X: {item.scenePos().x():.1f}, Y: {item.scenePos().y():.1f}")
            with QSignalBlocker(self.robot_x_input), QSignalBlocker(self.robot_y_input), QSignalBlocker(self.robot_heading_input), QSignalBlocker(self.robot_width_input), QSignalBlocker(self.robot_length_input):
                self.robot_x_input.setValue(item.scenePos().x())
                self.robot_y_input.setValue(item.scenePos().y())
                self.robot_heading_input.setValue(item.rotation())
                self.robot_width_input.setValue(item.robot.width)
                self.robot_length_input.setValue(item.robot.length)
            with QSignalBlocker(self.sensor_x_input), QSignalBlocker(self.sensor_y_input), QSignalBlocker(self.sensor_size_input):
                self.sensor_x_input.setValue(item.robot.sensor_x_mm)
                self.sensor_y_input.setValue(item.robot.sensor_y_mm)
                self.sensor_size_input.setValue(item.robot.sensor_diameter_mm)
        if item is None:
            self.selected_label.setText("Selected Item: None")
            self.distance_label.setText("Distance: —")
            self.angle_label.setText("Heading: —")
            self.turn_label.setText("Turn: —")
            self.movement_label.setText("Movement Type: —")
        if not isinstance(item, Waypoint):
            self.x_input.setEnabled(False)
            self.y_input.setEnabled(False)
        self.robot_group.setEnabled(isinstance(item, RobotPolygon))
        self.sensor_group.setEnabled(isinstance(item, RobotPolygon))
        self.movement_group.setEnabled(isinstance(item, PathSegment))
        self.intersection_group.setEnabled(isinstance(item, Waypoint) and any(segment.end_waypoint is item and segment.movement_type == "Line Follow (PID)" and segment.line_stop == "Intersection" for segment in item.connected_segments))
        line_follow = isinstance(item, PathSegment) and item.movement_type == "Line Follow (PID)"
        for control in (self.line_side_input, self.line_stop_input, self.line_target_input, self.kp_input, self.ki_input, self.kd_input):
            control.setEnabled(line_follow)
        self.robot_add_vertex_button.setEnabled(isinstance(item, RobotPolygon) and self.canvas.robot_item.edit_mode)
        self.robot_remove_vertex_button.setEnabled(isinstance(item, RobotPolygon) and self.canvas.robot_item.edit_mode and self.canvas.robot_item.selected_vertex is not None and len(self.canvas.robot_item.vertices) > 3)

    def updateIntersectionProperties(self, value=None):
        selected_items = self.canvas.scene.selectedItems()
        if len(selected_items) != 1 or not isinstance(selected_items[0], Waypoint):
            return
        waypoint = selected_items[0]
        waypoint.intersection_count = self.intersection_count_input.value()
        waypoint.intersection_direction = self.intersection_direction_input.currentText()
        if self.canvas.preview_playing:
            self.canvas.pauseRobotPreview()

    def updateMovementProperties(self, value=None):
        selected_items = self.canvas.scene.selectedItems()
        if len(selected_items) != 1 or not isinstance(selected_items[0], PathSegment):
            return
        segment = selected_items[0]
        segment.movement_type = self.movement_type_input.currentText()
        segment.speed_mm_s = self.speed_input.value()
        segment.turn_speed_deg_s = self.turn_speed_input.value()
        segment.line_side = self.line_side_input.currentText()
        segment.line_stop = self.line_stop_input.currentText()
        segment.line_target = self.line_target_input.value()
        segment.kp = self.kp_input.value()
        segment.ki = self.ki_input.value()
        segment.kd = self.kd_input.value()
        segment.updateMovementAppearance()
        self.movement_label.setText(f"Movement Type: {segment.movement_type}")
        self.intersection_group.setEnabled(False)
        for control in (self.line_side_input, self.line_stop_input, self.line_target_input, self.kp_input, self.ki_input, self.kd_input):
            control.setEnabled(segment.movement_type == "Line Follow (PID)")
        if self.canvas.preview_playing:
            self.canvas.pauseRobotPreview()

    def updateSensorProperties(self):
        robot = self.canvas.robot
        robot.sensor_x_mm = self.sensor_x_input.value()
        robot.sensor_y_mm = self.sensor_y_input.value()
        robot.sensor_diameter_mm = self.sensor_size_input.value()
        self.canvas.robot_item.updateSensorPosition()
        self.canvas.refreshRobotPreview()

    def updateRobotProperties(self):
        selected_items = self.canvas.scene.selectedItems()
        if len(selected_items) == 1 and isinstance(selected_items[0], RobotPolygon):
            item = selected_items[0]
            item.setPos(self.robot_x_input.value(), self.robot_y_input.value())
            item.setRotation(self.robot_heading_input.value())
            item.robot.width = self.robot_width_input.value()
            item.robot.length = self.robot_length_input.value()
            item.updateDimensions()

    def setRobotShapeEditing(self, enabled):
        self.canvas.robot_item.setEditMode(enabled)
        self.robot_add_vertex_button.setEnabled(enabled)
        self.robot_remove_vertex_button.setEnabled(enabled and self.canvas.robot_item.selected_vertex is not None and len(self.canvas.robot_item.vertices) > 3)

    def addRobotVertex(self):
        self.canvas.robot_item.addVertex()
        self.updateProperties(self.canvas.robot_item)

    def removeRobotVertex(self):
        self.canvas.robot_item.removeVertex()
        self.updateProperties(self.canvas.robot_item)

    def updatePreviewControls(self, item=None):
        count = len(self.canvas.waypoints)
        with QSignalBlocker(self.preview_index_input):
            self.preview_index_input.setMaximum(max(0, count - 1))
            self.preview_index_input.setValue(min(self.preview_index_input.value(), max(0, count - 1)))
        self.preview_button.setEnabled(count > 0)
        self.preview_index_input.setEnabled(count > 0)
        self.preview_mode_input.setEnabled(count > 0)
        self.preview_previous_button.setEnabled(count > 0 and self.preview_index_input.value() > 0)
        self.preview_next_button.setEnabled(count > 0 and self.preview_index_input.value() < count - 1)
        self.preview_play_button.setEnabled(count > 1)
        self.preview_reset_button.setEnabled(count > 0)
        if count == 0 and self.preview_button.isChecked():
            self.preview_button.setChecked(False)
        self.updateRobotPreview()

    def updateRobotPreview(self, value=None):
        self.preview_button.setText("Hide Preview" if self.preview_button.isChecked() else "Show Preview")
        self.canvas.setRobotPreview(self.preview_button.isChecked(), self.preview_index_input.value(), self.preview_mode_input.currentText())
        self.preview_pause_button.setEnabled(False)
        self.preview_play_button.setEnabled(len(self.canvas.waypoints) > 1)
        count = len(self.canvas.waypoints)
        self.preview_previous_button.setEnabled(count > 0 and self.preview_index_input.value() > 0)
        self.preview_next_button.setEnabled(count > 0 and self.preview_index_input.value() < count - 1)

    def playSmoothPreview(self):
        if not self.preview_button.isChecked():
            self.preview_button.setChecked(True)
        self.canvas.playRobotPreview()
        self.preview_play_button.setEnabled(not self.canvas.preview_playing)
        self.preview_pause_button.setEnabled(self.canvas.preview_playing)

    def pauseSmoothPreview(self):
        self.canvas.pauseRobotPreview()
        self.preview_play_button.setEnabled(len(self.canvas.waypoints) > 1)
        self.preview_pause_button.setEnabled(False)

    def resetSmoothPreview(self):
        self.canvas.resetRobotPreview()
        self.pauseSmoothPreview()
        with QSignalBlocker(self.preview_index_input):
            self.preview_index_input.setValue(0)

    def onPreviewProgress(self, index):
        with QSignalBlocker(self.preview_index_input):
            self.preview_index_input.setValue(index)
        self.preview_previous_button.setEnabled(index > 0)
        self.preview_next_button.setEnabled(index < len(self.canvas.waypoints) - 1)

    def onPreviewFinished(self):
        self.preview_play_button.setEnabled(len(self.canvas.waypoints) > 1)
        self.preview_pause_button.setEnabled(False)

    def updateWaypointPosition(self):
        selected_items = self.canvas.scene.selectedItems()
        if len(selected_items) == 1 and isinstance(selected_items[0], Waypoint):
            selected_items[0].setPos(self.x_input.value(), self.y_input.value())




if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
