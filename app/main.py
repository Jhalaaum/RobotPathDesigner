
import sys
import math
import ast
from navigation_ui import NavigationOverlay
from actions import Drive, Turn, LineTrack, TrackUntilIntersection, Program, from_pybricks, to_pybricks, to_movements
import copy
import json
from pathlib import Path
import random
from types import SimpleNamespace

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
    QGraphicsPathItem,
    QPushButton,
    QSpinBox,
    QComboBox,
    QScrollArea,
    QCheckBox,
    QSlider,
    QDialog,
    QHBoxLayout,
    QTextEdit,
    QMessageBox, QListWidget, QLineEdit, QGraphicsRectItem, QGraphicsSimpleTextItem, QColorDialog
)

from PySide6.QtPdf import QPdfDocument
from PySide6.QtGui import QPen, QBrush, QColor, QPainter, QPixmap, QAction, QActionGroup, QPolygonF, QImage, QPainterPath, QDrag, QKeySequence
from PySide6.QtCore import Qt, QSize, Signal, QSignalBlocker, QPointF, QTimer, QElapsedTimer, QMimeData

def generatePybricksRoute(waypoints, segments, robot):
    """Generate minimal movement calls for pasting into the existing Pybricks main.py."""
    warnings = []
    errors = []
    if len(waypoints) < 2:
        return "", warnings, ["Add at least two waypoints before exporting."]
    if len(segments) != len(waypoints) - 1:
        errors.append("The route must contain exactly one segment between each consecutive waypoint.")
    ordered_segments = []
    for index in range(len(waypoints) - 1):
        segment = next((item for item in segments if item.start_waypoint is waypoints[index] and item.end_waypoint is waypoints[index + 1]), None)
        if segment is None:
            errors.append(f"Missing segment from waypoint {waypoints[index].waypoint_id} to {waypoints[index + 1].waypoint_id}.")
        ordered_segments.append(segment)
    if errors:
        return "", warnings, errors

    if abs(robot.wheel_diameter - 62.4) > 0.1 or abs(robot.axle_track - 190) > 0.1:
        warnings.append("Robot dimensions in the planner differ from the competition-day code (wheel 62.4 mm, axle 190 mm).")

    actions = []
    heading = float(robot.initial_heading)
    def angle_difference(a, b):
        return (a - b + 180.0) % 360.0 - 180.0
    def format_number(number):
        return str(int(round(number))) if abs(number - round(number)) < 0.00001 else f"{number:.2f}".rstrip("0").rstrip(".")

    for index, segment in enumerate(ordered_segments):
        start = waypoints[index]
        end = waypoints[index + 1]
        distance_mm = float(segment.distance) * 10.0
        signed_distance_mm = -distance_mm if segment.reverse else distance_mm
        path_heading = float(segment.angle_degrees) if segment.angle_degrees is not None else heading
        target_heading = (path_heading + 180.0) % 360.0 if segment.reverse else path_heading
        route_delta = angle_difference(target_heading, heading)
        incoming = ordered_segments[index - 1] if index else None
        if start.turn_override_enabled:
            requested = float(start.turn_angle_deg)
        elif incoming is not None and incoming.movement_type == "Line Follow (PID)" and incoming.line_stop == "Intersection" and start.intersection_direction != "Follow Route":
            requested = {"Left": -90.0, "Right": 90.0, "Straight": 0.0}[start.intersection_direction]
        else:
            requested = route_delta
        clip = max(0.0, float(start.turn_clip_deg))
        clipped = max(-clip, min(clip, requested))
        if abs(clipped - requested) > 0.1:
            warnings.append(f"Waypoint {start.waypoint_id}: requested turn clipped from {requested:g} to {clipped:g} degrees.")
        if abs(clipped - route_delta) > 5:
            warnings.append(f"Waypoint {start.waypoint_id}: turn differs from the drawn route; check robot orientation.")
        if abs(clipped) >= 0.05:
            actions.append(Turn(float(format_number(clipped))))
        heading = target_heading

        if segment.movement_type == "Drive Straight":
            if distance_mm >= 0.5:
                actions.append(Drive(float(format_number(signed_distance_mm))))
            else:
                warnings.append(f"Segment {start.waypoint_id}->{end.waypoint_id} has negligible distance.")
        elif segment.movement_type == "Line Follow (PID)":
            if segment.line_side not in ("left", "right"):
                errors.append(f"Segment {start.waypoint_id}->{end.waypoint_id} has an unknown line side.")
                continue
            if segment.line_stop == "Distance":
                actions.append(LineTrack(float(format_number(signed_distance_mm)), segment.line_side))
            elif segment.line_stop == "Intersection":
                if int(end.intersection_count) != 1:
                    errors.append(f"Waypoint {end.waypoint_id} requests intersection #{end.intersection_count}, but the Pybricks helper stops at the first intersection.")
                actions.append(TrackUntilIntersection(segment.line_side))
                warnings.append(f"Waypoint {end.waypoint_id}: no automatic post-detection sensor-to-centre advance is exported; add a calibrated movement before turning if needed.")
            else:
                errors.append(f"Segment {start.waypoint_id}->{end.waypoint_id} has an unknown line stop.")
            if float(segment.ki) != 0 or abs(float(segment.kp) - 4.0) > 0.001 or abs(float(segment.kd) - 8.0) > 0.001 or abs(float(segment.line_target) - 48.5) > 0.001:
                warnings.append(f"Segment {start.waypoint_id}->{end.waypoint_id}: custom PID/reflectance settings are not exported; existing robot constants apply.")
            if abs(float(segment.speed_mm_s) - 200.0) > 0.001:
                warnings.append(f"Segment {start.waypoint_id}->{end.waypoint_id}: planner speed is not exported; the robot helper's default speed applies.")
        else:
            errors.append(f"Segment {start.waypoint_id}->{end.waypoint_id}: unsupported movement {segment.movement_type!r}.")

    final = waypoints[-1]
    if final.turn_override_enabled and abs(float(final.turn_angle_deg)) > 0.05:
        angle = max(-float(final.turn_clip_deg), min(float(final.turn_clip_deg), float(final.turn_angle_deg)))
        actions.append(Turn(float(format_number(angle))))
    warnings.append("The output contains only movement calls; paste them into your existing Pybricks program after its helper definitions.")
    code = ""
    try:
        code = to_pybricks(Program(tuple(actions)))
        compile(code, "<minimal_pybricks_route>", "exec")
    except (SyntaxError, ValueError) as exc:
        errors.append(f"Generated action error: {exc}")
    return code, warnings, errors

def parsePybricksRoute(code, intersection_distance_mm=200.0, auto_intersections=False):
    """Parse through the shared action model; never execute pasted code."""
    try:
        program = from_pybricks(code)
        statements = [stmt for stmt in ast.parse(code).body if not isinstance(stmt, ast.Pass)]
    except SyntaxError as exc:
        raise ValueError(f"Python syntax error on line {exc.lineno}: {exc.msg}") from exc
    movements, warnings = to_movements(program, intersection_distance_mm, auto_intersections)
    source_lines = [stmt.lineno for stmt, action in zip(statements, program.actions) if not isinstance(action, (Drive, LineTrack)) or abs(action.distance) >= 0.001]
    for movement, source_line in zip(movements, source_lines):
        if movement.get("estimated"):
            movement["source_line"] = source_line
    return movements, warnings

PART_TYPES = {
    'Wheel': (26, 56, '#64748b'), 'Drive motor': (48, 32, '#64748b'),
    'Colour sensor': (24, 24, '#eab308'), 'Distance sensor': (32, 24, '#0ea5e9'),
    'Force sensor': (24, 24, '#f97316'), 'Hub / gyro': (72, 64, '#94a3b8'),
    'Attachment motor': (48, 32, '#8b5cf6'), 'Gripper': (65, 60, '#a855f7'),
    'Forklift': (80, 60, '#a855f7'), 'Pusher': (24, 100, '#a855f7'),
    'Rotating arm': (90, 24, '#a855f7'), 'Linear slider': (85, 30, '#a855f7'),
    'Cargo bay': (65, 65, '#14b8a6'), 'Custom block': (50, 50, '#6b7280'),
}

def newRobotPart(kind, x=0, y=0):
    length, width, _ = PART_TYPES[kind]
    return dict(kind=kind, name=kind, x=float(x), y=float(y), length=float(length), width=float(width), angle=0.0, port='', direction='Clockwise', gear_ratio=1.0, sensing=18.0, primary=False)

class BuilderPalette(QListWidget):
    def startDrag(self, actions):
        item = self.currentItem()
        if item is None:
            return
        mime = QMimeData()
        mime.setText(item.text())
        drag = QDrag(self)
        drag.setMimeData(mime)
        drag.exec(Qt.DropAction.CopyAction)

class BuilderView(QGraphicsView):
    def __init__(self, editor, scene):
        super().__init__(scene)
        self.editor = editor
        self.setAcceptDrops(True)
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setMinimumSize(420, 420)
        self.setBackgroundBrush(QColor('#f1f5f9'))

    def dragEnterEvent(self, event):
        if event.mimeData().text() in PART_TYPES:
            event.acceptProposedAction()
        else:
            super().dragEnterEvent(event)

    def dragMoveEvent(self, event):
        if event.mimeData().text() in PART_TYPES:
            event.acceptProposedAction()
        else:
            super().dragMoveEvent(event)

    def dropEvent(self, event):
        kind = event.mimeData().text()
        if kind in PART_TYPES:
            p = self.mapToScene(event.position().toPoint())
            self.editor.addPart(kind, p.x(), p.y())
            event.acceptProposedAction()
        else:
            super().dropEvent(event)

    def drawBackground(self, painter, rect):
        super().drawBackground(painter, rect)
        painter.setPen(QPen(QColor('#dbe3ed'), 0))
        for x in range(math.floor(rect.left()/25)*25, math.ceil(rect.right()), 25):
            painter.drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()))
        for y in range(math.floor(rect.top()/25)*25, math.ceil(rect.bottom()), 25):
            painter.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))
        painter.setPen(QPen(QColor('#94a3b8'), 0, Qt.PenStyle.DashLine))
        painter.drawLine(QPointF(rect.left(), 0), QPointF(rect.right(), 0))
        painter.drawLine(QPointF(0, rect.top()), QPointF(0, rect.bottom()))

class BuilderPartItem(QGraphicsRectItem):
    def __init__(self, editor, part):
        super().__init__()
        self.editor, self.part = editor, part
        self.setFlags(QGraphicsItem.GraphicsItemFlag.ItemIsMovable | QGraphicsItem.GraphicsItemFlag.ItemIsSelectable | QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges)
        self.setZValue(5)
        self.label = QGraphicsSimpleTextItem(self)
        self.label.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self.label.setBrush(QColor('#172033'))
        self.syncing = False
        self.refresh()

    def refresh(self):
        self.syncing = True
        p = self.part
        self.setRect(-p['length']/2, -p['width']/2, p['length'], p['width'])
        c = QColor(PART_TYPES[p['kind']][2]); c.setAlpha(175)
        self.setBrush(c)
        self.setPen(QPen(QColor('#334155'), 1))
        self.label.setText(p['name'] + (' ★' if p['primary'] else ''))
        self.label.setPos(-p['length']/2, -p['width']/2-17)
        self.setPos(p['x'], p['y']); self.setRotation(p['angle'])
        self.syncing = False

    def itemChange(self, change, value):
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged and hasattr(self, 'syncing') and not self.syncing:
            self.part['x'], self.part['y'] = value.x(), value.y()
            if self.isSelected():
                self.editor.readSelection()
        return super().itemChange(change, value)

class RobotBuilder(QDialog):
    def __init__(self, canvas, parent=None):
        super().__init__(parent)
        self.canvas = canvas
        self.setWindowTitle('Robot Builder')
        self.resize(1050, 650)
        r = canvas.robot
        self.parts = copy.deepcopy(getattr(r, 'parts', []))
        if not self.parts:
            p = newRobotPart('Colour sensor', r.sensor_x_mm, r.sensor_y_mm); p.update(primary=True, sensing=r.sensor_diameter_mm)
            self.parts = [p]
        else:
            for p in self.parts:
                if p['primary']:
                    p.update(x=r.sensor_x_mm, y=r.sensor_y_mm, sensing=r.sensor_diameter_mm)
        self.vertices = [[p.x()/canvas.pixels_per_mm, p.y()/canvas.pixels_per_mm] for p in canvas.robot_item.vertices]
        self.items = []
        self.loading = False
        root = QVBoxLayout(self)
        root.addWidget(QLabel('Drag parts onto your robot  •  Forward is +X (right)  •  Dimensions in mm'))
        body = QHBoxLayout(); root.addLayout(body, 1)
        left = QVBoxLayout(); body.addLayout(left)
        left.addWidget(QLabel('PARTS'))
        self.palette = BuilderPalette(); self.palette.addItems(list(PART_TYPES)); self.palette.setDragEnabled(True); self.palette.setMaximumWidth(175)
        left.addWidget(self.palette)
        self.palette.itemDoubleClicked.connect(lambda item: self.addPart(item.text(), 0, 0))
        add = QPushButton('Add selected part'); left.addWidget(add); add.clicked.connect(lambda: self.addPart(self.palette.currentItem().text(), 0, 0) if self.palette.currentItem() else None)
        self.scene = QGraphicsScene(self); self.scene.setSceneRect(-400,-350,800,700)
        self.view = BuilderView(self, self.scene); body.addWidget(self.view, 1)
        self.chassis = QGraphicsPolygonItem(); self.chassis.setBrush(QColor(34,197,94,50)); self.chassis.setPen(QPen(QColor('#15803d'),2)); self.scene.addItem(self.chassis)
        self.chassis.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self.arrow = self.scene.addLine(0,0,65,0,QPen(QColor('#15803d'),3)); self.arrow.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        right = QVBoxLayout(); body.addLayout(right)
        hardware = QGroupBox('Chassis & drive'); right.addWidget(hardware); form = QFormLayout(hardware)
        self.drive = {}
        for key, label, value in [('length','Length',r.length),('width','Width',r.width),('wheel_diameter','Wheel diameter',r.wheel_diameter),('axle_track','Axle track',r.axle_track)]:
            c = QDoubleSpinBox(); c.setRange(1,1000); c.setDecimals(1); c.setSuffix(' mm'); c.setValue(value); form.addRow(label,c); self.drive[key]=c
        self.base_length = r.length; self.base_width = r.width
        self.drive['length'].valueChanged.connect(self.updateChassis); self.drive['width'].valueChanged.connect(self.updateChassis)
        self.updateChassis()
        self.group = QGroupBox('Selected part'); right.addWidget(self.group); f = QFormLayout(self.group)
        self.name = QLineEdit(); f.addRow('Name',self.name); self.name.textChanged.connect(self.writeSelection)
        self.controls = {}
        for key, label, low, high in [('x','Forward X',-1500,1500),('y','Right Y',-1500,1500),('length','Length',1,1000),('width','Width',1,1000),('angle','Rotation',-360,360),('gear_ratio','Gear ratio',0.01,100),('sensing','Optical diameter',1,250)]:
            c = QDoubleSpinBox(); c.setRange(low,high); c.setDecimals(2); c.setSuffix('°' if key=='angle' else ('' if key=='gear_ratio' else ' mm')); f.addRow(label,c); self.controls[key]=c; c.valueChanged.connect(self.writeSelection)
        self.port = QComboBox(); self.port.addItems(['','A','B','C','D','E','F']); f.addRow('Port',self.port); self.port.currentTextChanged.connect(self.writeSelection)
        self.direction = QComboBox(); self.direction.addItems(['Clockwise','Counterclockwise']); f.addRow('Direction',self.direction); self.direction.currentTextChanged.connect(self.writeSelection)
        self.primary = QPushButton('Use for line tracking'); f.addRow(self.primary); self.primary.clicked.connect(self.setPrimary)
        remove = QPushButton('Remove part'); f.addRow(remove); remove.clicked.connect(self.removePart)
        right.addStretch()
        bottom = QHBoxLayout(); root.addLayout(bottom)
        for label, handler in [('Load profile…',self.loadProfile),('Save profile…',self.saveProfile),('Cancel',self.reject),('Apply to robot',self.apply)]:
            b=QPushButton(label); bottom.addWidget(b); b.clicked.connect(handler)
        self.scene.selectionChanged.connect(self.readSelection)
        for p in self.parts: self.addItem(p)
        self.readSelection()
        QTimer.singleShot(0, lambda: self.view.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio))

    def updateChassis(self):
        sx=self.drive['length'].value()/self.base_length; sy=self.drive['width'].value()/self.base_width
        self.chassis.setPolygon(QPolygonF([QPointF(x*sx,y*sy) for x,y in self.vertices]))

    def addItem(self,p):
        item=BuilderPartItem(self,p); self.scene.addItem(item); self.items.append(item); return item

    def addPart(self,kind,x,y):
        p=newRobotPart(kind,x,y); self.parts.append(p); item=self.addItem(p); self.scene.clearSelection(); item.setSelected(True)

    def selected(self):
        return next((i for i in self.scene.selectedItems() if isinstance(i,BuilderPartItem)),None)

    def readSelection(self):
        item=self.selected(); self.group.setEnabled(item is not None)
        if item is None: return
        self.loading=True
        try:
            p=item.part; self.name.setText(p['name']); self.port.setCurrentText(p['port']); self.direction.setCurrentText(p['direction'])
            for k,c in self.controls.items(): c.setValue(p[k])
            self.primary.setEnabled(p['kind']=='Colour sensor'); self.primary.setText('Active line sensor ★' if p['primary'] else 'Use for line tracking')
            self.controls['sensing'].setEnabled(p['kind']=='Colour sensor')
        finally: self.loading=False

    def writeSelection(self,*args):
        item=self.selected()
        if self.loading or item is None: return
        p=item.part; p.update(name=self.name.text(),port=self.port.currentText(),direction=self.direction.currentText())
        p.update({k:c.value() for k,c in self.controls.items()}); item.refresh()

    def setPrimary(self):
        item=self.selected()
        if item is None or item.part['kind']!='Colour sensor': return
        for i in self.items: i.part['primary']=i is item; i.refresh()
        self.readSelection()

    def removePart(self):
        item=self.selected()
        if item is None:return
        self.items.remove(item); self.parts.remove(item.part); self.scene.removeItem(item)
        self.readSelection()

    def profile(self):
        return dict(version=1, drive={k:c.value() for k,c in self.drive.items()}, vertices=[[p.x(),p.y()] for p in self.chassis.polygon()], parts=copy.deepcopy(self.parts))

    def validate(self, data):
        if data['version']!=1: raise ValueError('Unsupported profile version')
        ports=[]; primary=0
        for k in self.drive:
            v=data['drive'][k]
            if not isinstance(v,(int,float)) or not math.isfinite(v) or not 1<=v<=1000: raise ValueError('Invalid drive dimension')
        if len(data['vertices'])<3 or any(len(p)!=2 or any(not isinstance(v,(int,float)) or not math.isfinite(v) for v in p) for p in data['vertices']): raise ValueError('Invalid chassis polygon')
        for p in data['parts']:
            if p['kind'] not in PART_TYPES or not isinstance(p['name'],str) or p['port'] not in ('','A','B','C','D','E','F') or p['direction'] not in ('Clockwise','Counterclockwise'): raise ValueError('Invalid part')
            for k,c in self.controls.items():
                v=p[k]
                if not isinstance(v,(int,float)) or not math.isfinite(v) or not c.minimum()<=v<=c.maximum(): raise ValueError('Invalid part dimension: '+k)
            if p['port']: ports.append(p['port'])
            if p['primary']:
                if p['kind']!='Colour sensor': raise ValueError('Line sensor must be a colour sensor')
                primary+=1
        if len(ports)!=len(set(ports)): raise ValueError('Each hardware port may only be assigned once')
        if primary!=1: raise ValueError('Select exactly one colour sensor for line tracking')

    def saveProfile(self):
        try:
            data=self.profile(); self.validate(data)
            path,_=QFileDialog.getSaveFileName(self,'Save robot profile','','Robot profile (*.json)')
            if path:
                with open(path,'w') as f: json.dump(data,f,indent=2)
        except Exception as e: QMessageBox.warning(self,'Robot profile',str(e))

    def loadProfile(self):
        path,_=QFileDialog.getOpenFileName(self,'Load robot profile','','Robot profile (*.json)')
        if not path:return
        try:
            with open(path) as f: data=json.load(f)
            self.validate(data)
        except Exception as e: QMessageBox.warning(self,'Robot profile',str(e)); return
        self.scene.clearSelection()
        for item in self.items:self.scene.removeItem(item)
        self.items=[]; self.parts=data['parts']; self.vertices=data['vertices']; self.base_length=data['drive']['length']; self.base_width=data['drive']['width']
        for k,c in self.drive.items():
            with QSignalBlocker(c): c.setValue(data['drive'][k])
        self.updateChassis()
        for p in self.parts:self.addItem(p)
        self.readSelection()

    def apply(self):
        try: data=self.profile(); self.validate(data)
        except Exception as e: QMessageBox.warning(self,'Robot Builder',str(e)); return
        c=self.canvas; r=c.robot
        c.pauseRobotPreview(); c.preview_paused=False
        for k,v in data['drive'].items():setattr(r,k,v)
        r.parts=copy.deepcopy(data['parts']); p=next(p for p in r.parts if p['primary'])
        r.sensor_x_mm=p['x']; r.sensor_y_mm=p['y']; r.sensor_diameter_mm=p['sensing']
        c.robot_item.vertices=[QPointF(x*c.pixels_per_mm,y*c.pixels_per_mm) for x,y in data['vertices']]
        c.robot_item.rebuildHandles(); c.robot_item.refreshShape(); c.robot_item.updateSensorPosition()
        c.refreshBuilderParts(); c.refreshRobotPreview(); self.accept()


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
        self.parts = []

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
        self.turn_override_enabled = False
        self.turn_angle_deg = 0.0
        self.turn_clip_deg = 180.0

        self.setPen(QPen(QColor("blue")))
        self.setBrush(QBrush(QColor("blue")))
        self.setZValue(3)

        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges, True)

        self.setPos(x, y)

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        if self.scene() is not None:
            canvas = self.scene().parent()
            if canvas.auto_snap_intersections:
                canvas.snapWaypointToIntersection(self, quiet=True)

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
        self.reverse = False
        self.imported_estimate = False

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
        pen = QPen(QColor("darkgreen") if self.movement_type == "Line Follow (PID)" else QColor("blue"), 2)
        if self.reverse:
            pen.setStyle(Qt.PenStyle.DashLine)
        self.setPen(pen)

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

FIELD_TYPES = {
    "Movable block": (40, 40, "#d97706"), "Static obstacle": (80, 40, "#64748b"),
    "Barrier": (120, 15, "#475569"), "Container": (100, 80, "#0d9488"),
    "Target zone": (120, 100, "#16a34a"), "Colour marker": (30, 30, "#eab308"),
    "Stackable object": (50, 50, "#9333ea"), "Hinged object": (75, 30, "#2563eb"),
    "Sliding object": (75, 30, "#0891b2"), "Terrain region": (150, 100, "#a16207"),
    "Grid / matrix": (150, 120, "#db2777"), "Custom object": (60, 60, "#64748b")
}

def senior2026Objects(scale):
    """A reproducible geometric test fixture; coordinates refer to the 1600px field."""
    objects = []
    def add(kind, name, x, y, l, w, color, height=0, angle=0):
        objects.append(dict(kind=kind, name=name, x=x/scale, y=y/scale, length=l/scale, width=w/scale, height=height, angle=angle, color=color))
    colors = {"White":"#ffffff", "Green":"#21864a", "Blue":"#245bbb", "Yellow":"#f4d526"}
    for name, y in [("White",118),("Green",226),("Blue",334),("Yellow",442)]:
        for row in range(2):
            for col in range(3):
                add("Movable block", f"{name[0]}T{row*3+col+1}", 83+43*col, y+43*row, 21,21,colors[name], 16)
    rng = random.Random(2026)
    for name,y in [("Yellow",58),("Green",228),("Blue",398),("White",567)]:
        for row in range(5):
            for col in range(2):
                add("Movable block",f"{name[0]}C{row*2+col+1}",1490+44*col+rng.uniform(-4,4),y+21+26*row+rng.uniform(-3,3),29,7,colors[name],9.6,rng.uniform(-18,18))
    for name, x,y,l,w in [("Blue",659,302,68,314),("Green",941,302,68,314),("Yellow",800,426,214,67),("White",800,178,214,68)]:
        add("Target zone",name+" cement target",x,y,l,w,colors[name])
    for name,x,y,l,w in [("Start",125,647,170,170),("Sponsor",406,716,140,110),("Parking",1100,452,202,196)]:
        add("Target zone",name,x,y,l,w,"#16a34a")
    add("Grid / matrix","Mosaic frame (3 x 4)",800,302,144,110,"#64748b",10)
    pattern=["Blue","Yellow","Yellow","Green","White","Blue","Green","White","Yellow","Green","Blue","White"]
    for row in range(3):
        for col in range(4):
            name=pattern[row*4+col]
            add("Colour marker",f"M{row+1}.{col+1}",746+36*col,266+36*row,28,28,colors[name])
    add("Custom object","Rectangular trowel",605,718,86,44,"#d9534f",35)
    add("Container","Cement bowl",800,719,33,33,"#e2e8f0",25)
    add("Custom object","Masonry trowel",1002,718,45,64,"#d9534f",35)
    for i,(x,y,angle,color) in enumerate([(603,118,0,"#c43838"),(988,97,0,"#20252e"),(602,507,0,"#20252e"),(987,508,0,"#c43838")],1):
        add("Barrier",f"Barrier {i}",x,y,72,13,color,65,angle)
    return objects


def senior2026Parts():
    parts=[]
    for kind,name,x,y,port in [("Hub / gyro","Prime Hub",0,0,""),("Wheel","Left wheel",0,-95,""),("Wheel","Right wheel",0,95,""),("Drive motor","Left drive",-10,-65,"B"),("Drive motor","Right drive",-10,65,"F"),("Colour sensor","Line / mosaic sensor",80,0,"E"),("Attachment motor","Claw motor",60,-45,"A"),("Attachment motor","Deposit motor",-60,0,"C"),("Attachment motor","Slider motor",60,45,"D"),("Gripper","Claw",125,0,""),("Linear slider","Scanner carriage",90,0,""),("Cargo bay","Tile storage",-65,0,"")]:
        p=newRobotPart(kind,x,y); p.update(name=name,port=port,primary=kind=="Colour sensor")
        if port=="B": p["direction"]="Counterclockwise"
        if kind=="Wheel": p.update(length=62.4,width=26.0)
        parts.append(p)
    return parts


class FieldPalette(BuilderPalette):
    def startDrag(self, actions):
        item = self.currentItem()
        if item is None: return
        mime = QMimeData(); mime.setData("application/x-wro-field-part", item.text().encode())
        drag = QDrag(self); drag.setMimeData(mime); drag.exec(Qt.DropAction.CopyAction)

class FieldObject(QGraphicsRectItem):
    def __init__(self, canvas, data):
        super().__init__(); self.canvas=canvas; self.data=copy.deepcopy(data); self.syncing=True
        self.setFlags(QGraphicsItem.GraphicsItemFlag.ItemIsMovable | QGraphicsItem.GraphicsItemFlag.ItemIsSelectable | QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges)
        self.label_item=QGraphicsSimpleTextItem(self); self.label_item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self.refresh(); self.syncing=False

    def refresh(self):
        old=self.syncing; self.syncing=True
        p=self.data; scale=self.canvas.pixels_per_mm; l=p['length']*scale; w=p['width']*scale
        self.setRect(-l/2,-w/2,l,w); color=QColor(p['color']); color.setAlpha(45 if p['kind'] in ('Target zone','Terrain region','Grid / matrix') else 155)
        self.setBrush(QBrush(color)); pen=QPen(QColor(p['color']),2)
        if p['kind'] in ('Target zone','Terrain region','Grid / matrix'): pen.setStyle(Qt.PenStyle.DashLine)
        self.setPen(pen); self.setZValue(0.8 if p['kind'] in ('Target zone','Terrain region','Grid / matrix') else 2.5)
        self.setPos(p['x']*scale,p['y']*scale); self.setRotation(p['angle'])
        self.label_item.setText(p['name']); self.label_item.setBrush(QBrush(QColor('#334155'))); self.label_item.setPos(-l/2,-w/2-self.label_item.boundingRect().height())
        self.syncing=old

    def itemChange(self, change, value):
        if change==QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged and not getattr(self,'syncing',True):
            self.data['x']=value.x()/self.canvas.pixels_per_mm; self.data['y']=value.y()/self.canvas.pixels_per_mm
            self.canvas.itemSelected.emit(self)
        return super().itemChange(change,value)

class RobotCanvas(QGraphicsView):
    itemSelected = Signal(object)
    geometryChanged = Signal(object)
    previewProgress = Signal(int)
    previewFinished = Signal()
    previewStopped = Signal()
    sensorReading = Signal(str)
    debugUpdated = Signal(object)
    def __init__(self):
        super().__init__()
        
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)

        self.setWindowTitle("Robot Path Designer")
        self.resize(1600, 774)

        self.next_waypoint_id = 0
        self.waypoints = []
        self.segments = []
        self.field_objects = []
        self.setAcceptDrops(True)

        self.pixels_per_mm = 0.67739204064
        self.show_coordinate_grid = True
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
        self.preview_speed_multiplier = 1.0
        self.debug_left_dps = None
        self.debug_right_dps = None
        self.debug_error = None
        self.debug_target = None
        self.debug_status = "Ready"
        self.debug_elapsed = 0.0
        self.trail_enabled = True
        self.trail_path = QPainterPath()
        self.trail_item = QGraphicsPathItem()
        self.trail_item.setPen(QPen(QColor(218, 60, 205, 220), 2))
        self.trail_item.setZValue(2.7)
        self.trail_item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self.scene.addItem(self.trail_item)
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
        self.track_mask = None
        self.auto_snap_intersections = True
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
            elif isinstance(item, (RobotPolygon, FieldObject)):
                self.itemSelected.emit(item)
    def keyPressEvent(self, event):
        if event.key() in (Qt.Key.Key_Delete, Qt.Key.Key_Backspace):
            selected_items = self.scene.selectedItems()

            for item in selected_items:
                if isinstance(item, FieldObject):
                    self.field_objects.remove(item); self.scene.removeItem(item)
                elif isinstance(item, Waypoint):
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

    def drawForeground(self, painter, rect):
        super().drawForeground(painter, rect)
        if not getattr(self, "show_coordinate_grid", False) or not self.waypoints:
            return
        origin = self.waypoints[0].scenePos()
        spacing = self.pixels_per_mm * 100.0
        if spacing <= 0:
            return
        painter.save()
        painter.setPen(QPen(QColor(60, 100, 150, 85), 0))
        left_index = max(-100, math.floor((rect.left() - origin.x()) / spacing))
        right_index = min(100, math.ceil((rect.right() - origin.x()) / spacing))
        top_index = max(-100, math.floor((rect.top() - origin.y()) / spacing))
        bottom_index = min(100, math.ceil((rect.bottom() - origin.y()) / spacing))
        for i in range(left_index, right_index + 1):
            x = origin.x() + i * spacing
            painter.drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()))
        for i in range(top_index, bottom_index + 1):
            y = origin.y() + i * spacing
            painter.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))
        painter.setPen(QPen(QColor(25, 90, 170, 190), 0))
        painter.drawLine(QPointF(origin.x(), rect.top()), QPointF(origin.x(), rect.bottom()))
        painter.drawLine(QPointF(rect.left(), origin.y()), QPointF(rect.right(), origin.y()))
        painter.drawText(QPointF(origin.x() + 7, origin.y() - 7), "(0, 0) mm")
        painter.restore()

    def locateNextImportedIntersection(self, start_scene, heading_deg, side, max_distance_mm=2600.0):
        """Find the nearest T/+ junction in the forward corridor of a code-imported line track.

        Uses the existing PDF dark-pixel mask and junction classifier; it does not
        execute Pybricks code or alter the field image. Designed for straight approaches.
        """
        if self.background is None or self.background_image is None or self.buildIntersectionMask() is None:
            return None
        angle = math.radians(heading_deg)
        fx, fy = math.cos(angle), math.sin(angle)
        rx, ry = -fy, fx
        # Sample ahead of the sensor, whose mounting position may not coincide
        # with the robot centre. Junctions must still lie on the incoming track.
        sensor_offset = QPointF((fx * self.robot.sensor_x_mm + rx * self.robot.sensor_y_mm) * self.pixels_per_mm, (fy * self.robot.sensor_x_mm + ry * self.robot.sensor_y_mm) * self.pixels_per_mm)
        sensor_start = start_scene + sensor_offset
        image_bounds = self.background.sceneBoundingRect()
        seen = set()
        candidates = []
        for distance_mm in range(45, int(max_distance_mm) + 1, 90):
            probe = sensor_start + QPointF(fx * distance_mm * self.pixels_per_mm, fy * distance_mm * self.pixels_per_mm)
            if not image_bounds.contains(probe):
                break
            junction = self.findIntersectionNear(probe, radius_mm=62.0)
            if junction is None:
                continue
            diff = junction - start_scene
            forward_mm = (diff.x() * fx + diff.y() * fy) / self.pixels_per_mm
            lateral_mm = (-diff.x() * fy + diff.y() * fx) / self.pixels_per_mm
            # Keep the junction on the forward approach, not an adjacent black
            # line or a previous junction immediately behind the robot's sensor.
            if forward_mm < max(35.0, self.robot.sensor_x_mm * 0.25) or abs(lateral_mm - self.robot.sensor_y_mm) > 85.0:
                continue
            key = (round(junction.x() / 14), round(junction.y() / 14))
            if key in seen:
                continue
            seen.add(key)
            candidates.append((forward_mm, abs(lateral_mm - self.robot.sensor_y_mm), junction))
        candidates.sort(key=lambda item: (item[0], item[1]))
        for _, _, junction in candidates:
            incoming = SimpleNamespace(start_waypoint=SimpleNamespace(scenePos=lambda: start_scene), line_side=side)
            aligned = self.edgeAlignedWaypointPosition(junction, incoming)
            if aligned is None:
                continue
            delta = aligned - start_scene
            forward = (delta.x() * fx + delta.y() * fy) / self.pixels_per_mm
            lateral = (-delta.x() * fy + delta.y() * fx) / self.pixels_per_mm
            if forward >= 25 and abs(lateral) < 100:
                return aligned
        return None

    def resolveImportedIntersections(self, movements, origin_scene=None, initial_heading=None):
        """Resolve code-imported intersection stops before modifying the route."""
        origin = QPointF(origin_scene if origin_scene is not None else (self.waypoints[0].scenePos() if self.waypoints else self.robot_item.scenePos()))
        heading = float(self.robot.initial_heading if initial_heading is None else initial_heading)
        position = QPointF(origin)
        warnings = []
        for movement in movements:
            if movement["type"] == "turn":
                heading += movement["angle"]
                continue
            if movement.get("auto_detect") and movement.get("stop") == "Intersection":
                found = self.locateNextImportedIntersection(position, heading, movement["side"])
                if found is None:
                    warnings.append(f"Line {movement.get('source_line', '?')}: no confident forward junction; used {movement['distance']:g} mm fallback. Place the start/heading on the correct track, or adjust the waypoint manually.")
                else:
                    movement["target_scene"] = QPointF(found)
                    movement["distance"] = math.hypot(found.x() - position.x(), found.y() - position.y()) / self.pixels_per_mm
                    movement["estimated"] = False
            if "target_scene" in movement:
                position = QPointF(movement["target_scene"])
            else:
                radians = math.radians(heading)
                position += QPointF(math.cos(radians) * movement["distance"] * self.pixels_per_mm, math.sin(radians) * movement["distance"] * self.pixels_per_mm)
        return warnings

    def loadActionProgram(self, program, intersection_distance_mm=200.0, auto_intersections=False, origin_scene=None, initial_heading=None):
        """Bridge for future automatic navigation; current canvas is navigation-only."""
        movements, warnings = to_movements(program, intersection_distance_mm, auto_intersections)
        if auto_intersections:
            warnings.extend(self.resolveImportedIntersections(movements, origin_scene, initial_heading))
        self.loadRouteMovements(movements, origin_scene, initial_heading)
        return warnings

    def loadRouteMovements(self, movements, origin_scene=None, initial_heading=None):
        """Replace the editable route from parsed calls (millimetres, heading in screen degrees)."""
        self.resetRobotPreview()
        if origin_scene is None:
            origin_scene = self.waypoints[0].scenePos() if self.waypoints else self.robot_item.scenePos()
        if initial_heading is None:
            initial_heading = self.robot.initial_heading
        for segment in list(self.segments):
            self.removeSegment(segment)
        for waypoint in list(self.waypoints):
            self.scene.removeItem(waypoint)
        self.waypoints.clear()
        self.next_waypoint_id = 0
        self.robot.initial_heading = float(initial_heading)
        self.robot_item.setRotation(float(initial_heading))
        position = QPointF(origin_scene)
        heading = float(initial_heading)
        def add_waypoint(pos):
            wp = Waypoint(self.next_waypoint_id, pos.x(), pos.y())
            self.next_waypoint_id += 1
            self.waypoints.append(wp)
            self.scene.addItem(wp)
            return wp
        current = add_waypoint(position)
        for move in movements:
            if move["type"] == "turn":
                angle = move["angle"]
                current.turn_override_enabled = True
                current.turn_angle_deg += angle
                current.turn_clip_deg = max(180.0, abs(current.turn_angle_deg))
                heading += angle
                continue
            distance = move["distance"]
            radians = math.radians(heading)
            position = QPointF(move["target_scene"]) if "target_scene" in move else position + QPointF(math.cos(radians) * distance * self.pixels_per_mm, math.sin(radians) * distance * self.pixels_per_mm)
            next_wp = add_waypoint(position)
            segment = PathSegment(current, next_wp, self.pixels_per_mm)
            segment.imported_estimate = move["estimated"]
            segment.reverse = distance < 0
            if move["type"] == "line":
                segment.movement_type = "Line Follow (PID)"
                segment.line_side = move["side"]
                segment.line_stop = move["stop"]
            segment.updateMovementAppearance()
            self.segments.append(segment)
            self.scene.addItem(segment)
            current = next_wp
        self.robot_item.setPos(self.waypoints[0].scenePos())
        self.scene.clearSelection()
        self.refreshRobotPreview()
        self.geometryChanged.emit(self.waypoints[0])
        self.viewport().update()

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

        if item is not None and not isinstance(item, (QGraphicsPixmapItem, FieldObject)):
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
    def addFieldObject(self, kind, point):
        l,w,color=FIELD_TYPES[kind]; scale=self.pixels_per_mm
        data=dict(kind=kind,name=kind,length=l,width=w,height=0.0,color=color,x=point.x()/scale,y=point.y()/scale,angle=0.0)
        item=FieldObject(self,data); self.field_objects.append(item); self.scene.addItem(item); self.scene.clearSelection(); item.setSelected(True)
        return item

    def dragEnterEvent(self,event):
        if event.mimeData().hasFormat("application/x-wro-field-part"): event.acceptProposedAction()
        else: super().dragEnterEvent(event)

    def dragMoveEvent(self,event):
        if event.mimeData().hasFormat("application/x-wro-field-part"): event.acceptProposedAction()
        else: super().dragMoveEvent(event)

    def dropEvent(self,event):
        kind=bytes(event.mimeData().data("application/x-wro-field-part")).decode() if event.mimeData().hasFormat("application/x-wro-field-part") else None
        if kind in FIELD_TYPES:
            self.addFieldObject(kind,self.mapToScene(event.position().toPoint())); event.acceptProposedAction()
        else: super().dropEvent(event)

    def editSnapshot(self):
        wp_keys=('intersection_count','intersection_direction','turn_override_enabled','turn_angle_deg','turn_clip_deg')
        seg_keys=('movement_type','speed_mm_s','turn_speed_deg_s','line_target','kp','ki','kd','line_side','line_stop','reverse','imported_estimate')
        return dict(robot=copy.deepcopy(vars(self.robot)), pose=(self.robot_item.pos().x(),self.robot_item.pos().y(),self.robot_item.rotation()), vertices=[(p.x(),p.y()) for p in self.robot_item.vertices], next_id=self.next_waypoint_id, waypoints=[dict(id=w.waypoint_id,x=w.pos().x(),y=w.pos().y(),label=w.label,attrs={k:getattr(w,k) for k in wp_keys}) for w in self.waypoints], segments=[dict(start=t.start_waypoint.waypoint_id,end=t.end_waypoint.waypoint_id,attrs={k:getattr(t,k) for k in seg_keys}) for t in self.segments], objects=[copy.deepcopy(o.data) for o in self.field_objects])

    def restoreEditSnapshot(self,data):
        self.resetRobotPreview(); self.syncing_start=True
        try:
            with QSignalBlocker(self), QSignalBlocker(self.scene):
                self.scene.clearSelection()
                for t in list(self.segments): self.removeSegment(t)
                for w in self.waypoints: self.scene.removeItem(w)
                for o in self.field_objects: self.scene.removeItem(o)
                self.waypoints=[]; self.field_objects=[]
                vars(self.robot).update(copy.deepcopy(data['robot']))
                self.robot_item.vertices=[QPointF(x,y) for x,y in data['vertices']]
                self.robot_item.selected_vertex=None; self.robot_item.rebuildHandles(); self.robot_item.refreshShape(); self.robot_item.updateSensorPosition()
                x,y,a=data['pose']; self.robot_item.setPos(x,y); self.robot_item.setRotation(a); self.refreshBuilderParts()
                by_id={}
                for d in data['waypoints']:
                    w=Waypoint(d['id'],d['x'],d['y'],label=d['label']); by_id[d['id']]=w
                    for k,v in d['attrs'].items(): setattr(w,k,v)
                    self.waypoints.append(w); self.scene.addItem(w)
                for d in data['segments']:
                    t=PathSegment(by_id[d['start']],by_id[d['end']],self.pixels_per_mm)
                    for k,v in d['attrs'].items(): setattr(t,k,v)
                    t.updateMovementAppearance(); self.segments.append(t); self.scene.addItem(t)
                for d in data['objects']:
                    o=FieldObject(self,d); self.field_objects.append(o); self.scene.addItem(o)
                self.next_waypoint_id=data['next_id']
        finally: self.syncing_start=False
        self.refreshRobotPreview(); self.geometryChanged.emit(None); self.onSelectionChanged(); self.viewport().update()

    def importField(self, file_path=None):
        if not isinstance(file_path, (str, Path)):
            file_path, selected_filter = QFileDialog.getOpenFileName(self, "Import WRO Field", "", "PDF Files (*.pdf)")
        if file_path:
            file_path = str(file_path)
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

            overlay = getattr(self, "navigation_overlay", None)
            if overlay is not None: overlay.clear()
            if self.background is not None:
                self.scene.removeItem(self.background)

            self.background = item
            self.background_image = image
            self.track_mask = None
            if self.track_debug_item is not None: self.scene.removeItem(self.track_debug_item)
            self.track_debug_item = None
            self.scene.addItem(item)
            if overlay is not None: overlay.field_changed()
            if self.track_debug_enabled:
                self.updateTrackDebugOverlay()
            self.scene.setSceneRect(self.scene.itemsBoundingRect())

            self.fitInView(item, Qt.AspectRatioMode.KeepAspectRatio)
            self.zoom_level = self.transform().m11()
            return True
        else:
            print("No file selected")
        return False

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

    def buildIntersectionMask(self):
        """Cache the same dark-pixel predicate used by the sensor/debug overlay at half resolution."""
        if self.background_image is None:
            return None
        if self.track_mask is not None:
            return self.track_mask
        image = self.background_image
        width, height = (image.width() + 1) // 2, (image.height() + 1) // 2
        mask = bytearray(width * height)
        for y in range(height):
            for x in range(width):
                color = image.pixelColor(min(x * 2, image.width() - 1), min(y * 2, image.height() - 1))
                dark = min(color.red(), color.green(), color.blue())
                bright = max(color.red(), color.green(), color.blue())
                mask[y * width + x] = int(bright < 110 and bright - dark < 45)
        self.track_mask = (width, height, mask)
        return self.track_mask

    def findIntersectionNear(self, scene_position, radius_mm=110.0):
        """Find a nearby T/+ junction by requiring three connected, narrow black arms.

        This is a geometric PDF locator, NOT a replacement for the runtime sensor detector.
        It intentionally rejects bends (only two arms) and broad solid dark patches.
        """
        data = self.buildIntersectionMask()
        if data is None or self.background is None:
            return None
        width, height, mask = data
        local = self.background.mapFromScene(scene_position)
        cx, cy = round(local.x() / 2), round(local.y() / 2)
        radius = max(5, round(radius_mm * self.pixels_per_mm / 2))
        radius = min(radius, 90)
        def black(x, y):
            return 0 <= x < width and 0 <= y < height and mask[y * width + x] == 1
        hits = []
        for y in range(max(22, cy - radius), min(height - 22, cy + radius + 1), 2):
            for x in range(max(22, cx - radius), min(width - 22, cx + radius + 1), 2):
                if (x - cx) ** 2 + (y - cy) ** 2 > radius * radius or not black(x, y):
                    continue
                arms = []
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    support = sum(any(black(x + dx * step + dy * shift, y + dy * step + dx * shift) for shift in (-2, 0, 2)) for step in (8, 13, 19))
                    arms.append(support >= 3)
                if sum(arms) < 3:
                    continue
                # A track is a thin strip: reject large dark filled rectangles.
                if sum(black(x + dx, y + dy) for dx, dy in ((12, 12), (-12, 12), (12, -12), (-12, -12))) >= 2:
                    continue
                hits.append((x, y, sum(arms)))
        if not hits:
            return None
        # Group neighboring qualifying pixels into a single junction candidate, then
        # choose the junction nearest the current waypoint rather than a distant crossing.
        clusters = []
        for x, y, score in hits:
            matches = [cluster for cluster in clusters if (x - cluster[0] / cluster[2]) ** 2 + (y - cluster[1] / cluster[2]) ** 2 < 14 ** 2]
            if matches:
                cluster = matches[0]
                cluster[0] += x
                cluster[1] += y
                cluster[2] += 1
                cluster[3] = max(cluster[3], score)
            else:
                clusters.append([float(x), float(y), 1, score])
        clusters.sort(key=lambda cluster: ((cluster[0] / cluster[2] - cx) ** 2 + (cluster[1] / cluster[2] - cy) ** 2, -cluster[2]))
        x, y, _, _ = clusters[0]
        count = clusters[0][2]
        return self.background.mapToScene(QPointF(2.0 * x / count, 2.0 * y / count))

    def edgeAlignedWaypointPosition(self, junction_centre, incoming):
        """Align the turn pivot to the selected edge at the junction's forward station.

        The waypoint is the robot's rotation centre, not its forward-mounted sensor.
        The light sensor triggers earlier, and the robot must keep driving to the pivot.
        """
        start = incoming.start_waypoint.scenePos()
        dx = junction_centre.x() - start.x()
        dy = junction_centre.y() - start.y()
        length = math.hypot(dx, dy)
        if length <= 1 or self.background is None or self.track_mask is None:
            return None
        forward_x, forward_y = dx / length, dy / length
        right_x, right_y = -forward_y, forward_x
        # Sample the incoming arm *before* the intersection so the cross-arm
        # does not make the stripe appear artificially wide.
        sample_back = min(length * 0.45, 65.0 * self.pixels_per_mm)
        approach = junction_centre - QPointF(forward_x * sample_back, forward_y * sample_back)
        width, height, mask = self.track_mask
        def black_at(point):
            local = self.background.mapFromScene(point)
            x, y = round(local.x() / 2), round(local.y() / 2)
            return 0 <= x < width and 0 <= y < height and bool(mask[y * width + x])
        # Find nearby black stripe centre, accounting for the route being
        # slightly off the black-track centreline.
        centre_shift = None
        for offset_mm in range(0, 46, 2):
            for sign in ((1, -1) if offset_mm else (1,)):
                offset = sign * offset_mm * self.pixels_per_mm
                probe = approach + QPointF(right_x * offset, right_y * offset)
                if black_at(probe):
                    centre_shift = offset
                    break
            if centre_shift is not None:
                break
        if centre_shift is None:
            return None
        # Walk to both edges; this estimates the midline rather than treating
        # the first black pixel as the line centre.
        def edge_distance(direction):
            last_black = 0.0
            for mm in range(1, 46):
                offset = centre_shift + direction * mm * self.pixels_per_mm
                if not black_at(approach + QPointF(right_x * offset, right_y * offset)):
                    return centre_shift + direction * (mm - 0.5) * self.pixels_per_mm
                last_black = offset
            return None
        left_edge = edge_distance(-1)
        right_edge = edge_distance(1)
        if left_edge is None or right_edge is None:
            return None
        # Reject very wide dark objects that are unlikely to be the thin route.
        stripe_width_mm = (right_edge - left_edge) / self.pixels_per_mm
        if not (5.0 <= stripe_width_mm <= 65.0):
            return None
        chosen = right_edge if incoming.line_side == "right" else left_edge
        # The selected line edge defines the desired LATERAL lane.
        # The robot centre must still reach the true junction for an in-place turn.
        edge_point = junction_centre + QPointF(right_x * chosen, right_y * chosen)
        sensor_y = self.robot.sensor_y_mm * self.pixels_per_mm
        # Only remove lateral sensor offset. The robot's rotation centre must
        # remain at the junction's longitudinal station, AFTER the sensor
        # first detects the black intersection ahead of the axle.
        lateral_sensor = QPointF(right_x * sensor_y, right_y * sensor_y)
        return edge_point - lateral_sensor

    def snapWaypointToIntersection(self, waypoint, quiet=False):
        if waypoint not in self.waypoints or self.background_image is None:
            if not quiet:
                self.sensorReading.emit("Import the game mat PDF before auto-aligning a waypoint")
            return False
        incoming = next((segment for segment in waypoint.connected_segments if segment.end_waypoint is waypoint and segment.movement_type == "Line Follow (PID)" and segment.line_stop == "Intersection"), None)
        if incoming is None:
            if not quiet:
                self.sensorReading.emit("Choose Line Follow (PID) and Intersection stop on the incoming segment first")
            return False
        centre = self.findIntersectionNear(waypoint.scenePos())
        if centre is None:
            if not quiet:
                self.sensorReading.emit(f"No confident T/cross junction near Waypoint {waypoint.waypoint_id}; position unchanged")
            return False
        position = self.edgeAlignedWaypointPosition(centre, incoming)
        if position is None:
            if not quiet:
                self.sensorReading.emit("Could not identify the selected track edge on the intersection approach; waypoint unchanged")
            return False
        waypoint.setPos(position)
        if not quiet:
            self.sensorReading.emit(f"Waypoint {waypoint.waypoint_id}: robot centre aligned for {incoming.line_side} edge and sensor offset")
        return True

    def snapAllIntersectionWaypoints(self):
        success = sum(self.snapWaypointToIntersection(wp, quiet=True) for wp in self.waypoints[1:])
        self.sensorReading.emit(f"Auto-aligned {success} intersection waypoint(s); others unchanged")

    def effectiveWaypointTurn(self, waypoint, route_heading, heading):
        """Resolve an editable relative turn and clip its magnitude at the waypoint."""
        route_delta = (route_heading - heading + 180) % 360 - 180
        incoming = next((candidate for candidate in waypoint.connected_segments if candidate.end_waypoint is waypoint and candidate.movement_type == "Line Follow (PID)" and candidate.line_stop == "Intersection"), None)
        if waypoint.turn_override_enabled:
            delta = waypoint.turn_angle_deg
        elif incoming is not None and waypoint.intersection_direction != "Follow Route":
            delta = {"Left": -90.0, "Right": 90.0, "Straight": 0.0}[waypoint.intersection_direction]
        else:
            delta = route_delta
        clipped_delta = max(-waypoint.turn_clip_deg, min(waypoint.turn_clip_deg, delta))
        return (heading + clipped_delta + 180) % 360 - 180

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

    def refreshBuilderParts(self):
        for parent in (self.robot_item, self.preview_item):
            for item in getattr(parent, 'builder_parts', []):
                if item.scene() is not None: item.scene().removeItem(item)
            parent.builder_parts = []
            for p in self.robot.parts:
                if p['primary']: continue
                scale = self.pixels_per_mm
                l, w = p['length']*scale, p['width']*scale
                item = QGraphicsRectItem(-l/2, -w/2, l, w, parent)
                color = QColor(PART_TYPES[p['kind']][2]); color.setAlpha(110)
                item.setBrush(QBrush(color)); item.setPen(QPen(QColor('#475569'), 1))
                item.setPos(p['x']*scale, p['y']*scale); item.setRotation(p['angle']); item.setZValue(2)
                item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
                parent.builder_parts.append(item)

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
            incoming_heading = self.robot.initial_heading if self.preview_index == 0 else (incoming.angle_degrees if incoming is not None else self.robot.initial_heading)
            if incoming_heading is None:
                incoming_heading = self.robot.initial_heading
            if self.preview_mode == "Arrival":
                heading = incoming_heading
            elif outgoing is None:
                heading = self.effectiveWaypointTurn(waypoint, incoming_heading, incoming_heading) if waypoint.turn_override_enabled else incoming_heading
            else:
                route_heading = outgoing.angle_degrees if outgoing.angle_degrees is not None else incoming_heading
                heading = self.effectiveWaypointTurn(waypoint, route_heading, incoming_heading)
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

    def clearSimulationTrail(self):
        self.trail_path = QPainterPath()
        self.trail_item.setPath(self.trail_path)
        if self.waypoints:
            start = self.preview_item.pos()
            self.trail_path.moveTo(start)
            self.trail_item.setPath(self.trail_path)
        self.publishDebug()

    def setTrailVisible(self, visible):
        self.trail_enabled = bool(visible)
        self.trail_item.setVisible(self.trail_enabled)

    def recordSimulationTrail(self):
        if not self.waypoints or self.trail_path.elementCount() == 0:
            return
        position = self.preview_item.pos()
        last = self.trail_path.currentPosition()
        if math.hypot(position.x() - last.x(), position.y() - last.y()) >= 0.4:
            self.trail_path.lineTo(position)
            self.trail_item.setPath(self.trail_path)

    def publishDebug(self):
        segment = next((s for s in self.segments if self.preview_segment_index < len(self.waypoints) - 1 and s.start_waypoint is self.waypoints[self.preview_segment_index] and s.end_waypoint is self.waypoints[self.preview_segment_index + 1]), None)
        reading = self.sampleTrackReflectance(self.preview_sensor.scenePos()) if self.background_image is not None and self.preview_enabled else None
        self.debugUpdated.emit({"reflection": reading, "target": segment.line_target if segment is not None and segment.movement_type == "Line Follow (PID)" else None, "error": self.debug_error if self.preview_phase == "drive" else None, "left_dps": self.debug_left_dps if self.preview_phase == "drive" else None, "right_dps": self.debug_right_dps if self.preview_phase == "drive" else None, "intersections": self.line_intersections_found, "intersection_target": segment.end_waypoint.intersection_count if segment is not None and segment.line_stop == "Intersection" and segment.movement_type == "Line Follow (PID)" else None, "heading": self.preview_heading, "phase": self.preview_phase, "status": self.debug_status, "elapsed": self.debug_elapsed, "segment": self.preview_segment_index, "multiplier": self.preview_speed_multiplier})

    def singleStepPreview(self):
        if len(self.waypoints) < 2:
            return
        if not self.preview_enabled:
            self.preview_enabled = True
        if not self.preview_paused and not self.preview_playing:
            self.playRobotPreview()
            if not self.preview_playing:
                return
            self.preview_timer.stop()
        elif not self.preview_playing:
            self.preview_playing = True
        self.preview_timer.stop()
        self.advanceRobotPreview(0.02 / max(0.01, self.preview_speed_multiplier))
        if self.preview_playing:
            self.preview_playing = False
            self.preview_paused = True
        self.publishDebug()

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
        self.debug_error = None
        self.debug_left_dps = None
        self.debug_right_dps = None
        self.debug_status = "Driving"

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
                self.debug_status = "Stopped: sensor outside mat"
                self.pauseRobotPreview()
                self.sensorReading.emit("Line tracking stopped: sensor outside imported PDF or no field loaded")
                return
            base_dps = (-1 if segment.reverse else 1) * segment.speed_mm_s * 360 / (math.pi * max(1.0, self.robot.wheel_diameter))
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
            self.debug_error = segment.line_target - reading
            self.debug_left_dps = left_dps
            self.debug_right_dps = right_dps
            self.debug_status = "Aligning at junction" if self.line_approach_phase else ("Crossing junction" if self.line_crossing_phase else "Tracking " + segment.line_side + " edge")
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
                    self.debug_status = "Stopped: intersection alignment failed"
                    self.pauseRobotPreview()
                    self.sensorReading.emit(f"Intersection alignment failed: waypoint {waypoint.waypoint_id} is not on the robot's forward trajectory (lateral error {lateral_error:.0f} mm)")
                    return
            elif segment.line_stop == "Distance" and self.line_travel_mm >= distance_mm:
                self.finishPreviewSegment()
                return
            if self.line_travel_mm >= max(distance_mm * 1.8 + 100, distance_mm + 150):
                self.debug_status = "Stopped: intersection not found"
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
            self.debug_elapsed = 0.0
            self.debug_status = "Starting"
            self.debug_error = None
            self.debug_left_dps = None
            self.debug_right_dps = None
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
        if self.debug_elapsed == 0.0:
            self.clearSimulationTrail()
        self.publishDebug()

    def pauseRobotPreview(self):
        self.preview_paused = self.preview_playing and self.preview_segment_index < len(self.waypoints) - 1
        self.preview_playing = False
        self.preview_timer.stop()
        if self.debug_status not in ("Finished", "Stopped: sensor outside mat", "Stopped: intersection alignment failed", "Stopped: intersection not found"):
            self.debug_status = "Paused" if self.preview_paused else "Stopped"
        self.previewStopped.emit()
        self.publishDebug()

    def resetRobotPreview(self):
        self.pauseRobotPreview()
        self.preview_paused = False
        self.preview_index = 0
        self.preview_mode = "Departure"
        self.refreshRobotPreview()
        self.previewProgress.emit(0)
        self.debug_elapsed = 0.0
        self.debug_error = None
        self.debug_left_dps = None
        self.debug_right_dps = None
        self.debug_status = "Ready"
        self.clearSimulationTrail()

    def advanceRobotPreview(self, step_seconds=None):
        if not self.preview_playing:
            return
        frame_seconds = min(self.preview_clock.restart() / 1000.0, 0.1) if step_seconds is None else step_seconds
        simulation_seconds = frame_seconds * self.preview_speed_multiplier
        self.debug_elapsed += simulation_seconds
        self._advanceRobotPreviewCore(simulation_seconds)
        self.recordSimulationTrail()
        self.publishDebug()

    def _advanceRobotPreviewCore(self, simulation_seconds):
        if not self.preview_playing:
            return
        if self.preview_phase == "final_turn":
            elapsed = simulation_seconds
            duration = abs(self.preview_final_delta) / max(1.0, self.preview_turn_speed_deg_s)
            self.preview_phase_progress = min(1.0, self.preview_phase_progress + elapsed / max(0.001, duration))
            self.preview_heading = self.preview_turn_start + self.preview_final_delta * self.preview_phase_progress
            self.preview_item.setRotation(self.preview_heading)
            self.updatePreviewSensor()
            if self.preview_phase_progress >= 1.0:
                self.preview_phase = "finished"
                self.pauseRobotPreview()
                self.preview_paused = False
                self.previewFinished.emit()
            return
        if self.preview_segment_index >= len(self.waypoints) - 1:
            self.pauseRobotPreview()
            return
        remaining = simulation_seconds
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
            segment = next((segment for segment in self.segments if segment.start_waypoint is self.waypoints[self.preview_segment_index] and segment.end_waypoint is self.waypoints[self.preview_segment_index + 1]), None)
            target_heading = math.degrees(math.atan2(dy, dx))
            if segment is not None and segment.reverse:
                target_heading = (target_heading + 180.0) % 360.0
            if self.preview_phase == "turn":
                self.debug_status = "Turning"
                self.debug_error = None
                self.debug_left_dps = None
                self.debug_right_dps = None
                if self.preview_phase_progress == 0:
                    self.preview_turn_start = self.preview_heading
                    origin = self.waypoints[self.preview_segment_index]
                    self.preview_turn_target = self.effectiveWaypointTurn(origin, target_heading, self.preview_heading)
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
                reverse = segment is not None and segment.reverse
                self.debug_status = "Reversing" if reverse else "Driving straight"
                self.debug_error = None
                speed_mm_s = segment.speed_mm_s if segment is not None else self.preview_speed_mm_s
                wheel_dps = (-1 if reverse else 1) * speed_mm_s * 360 / (math.pi * max(1.0, self.robot.wheel_diameter))
                self.debug_left_dps = wheel_dps
                self.debug_right_dps = wheel_dps
                duration = distance / (speed_mm_s * self.pixels_per_mm)
                available = (1.0 - self.preview_phase_progress) * duration
                taken = min(remaining, available)
                self.preview_phase_progress = min(1.0, self.preview_phase_progress + taken / duration)
                direction = math.radians(self.preview_heading)
                signed_distance = -distance if reverse else distance
                self.preview_item.setPos(self.preview_drive_start + QPointF(math.cos(direction) * signed_distance * self.preview_phase_progress, math.sin(direction) * signed_distance * self.preview_phase_progress))
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
            final = self.waypoints[-1]
            if final.turn_override_enabled and final.turn_clip_deg > 0 and abs(final.turn_angle_deg) > 0.001:
                self.preview_turn_start = self.preview_heading
                self.preview_final_delta = max(-final.turn_clip_deg, min(final.turn_clip_deg, final.turn_angle_deg))
                self.preview_phase = "final_turn"
                self.preview_phase_progress = 0.0
                return
            self.debug_status = "Finished"
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

        self.turn_group = QGroupBox("Waypoint Turn")
        turn_layout = QFormLayout()
        self.turn_override_input = QCheckBox("Use custom angle")
        self.turn_angle_input = QDoubleSpinBox()
        self.turn_angle_input.setRange(-180, 180)
        self.turn_angle_input.setDecimals(1)
        self.turn_angle_input.setSuffix(" °")
        self.turn_angle_input.setToolTip("Executed at this waypoint BEFORE the next segment. Negative = anticlockwise on screen, positive = clockwise.")
        self.turn_clip_input = QDoubleSpinBox()
        self.turn_clip_input.setRange(0, 180)
        self.turn_clip_input.setDecimals(1)
        self.turn_clip_input.setValue(180)
        self.turn_clip_input.setSuffix(" °")
        self.turn_clip_input.setToolTip("Maximum magnitude of the RELATIVE turn in preview. 180° leaves turns unrestricted")
        turn_layout.addRow(self.turn_override_input)
        turn_layout.addRow("Relative turn:", self.turn_angle_input)
        turn_layout.addRow("Maximum angle limit:", self.turn_clip_input)
        self.turn_group.setLayout(turn_layout)
        self.turn_group.setEnabled(False)
        properties_layout.addWidget(self.turn_group)
        self.turn_override_input.toggled.connect(self.updateTurnProperties)
        self.turn_angle_input.valueChanged.connect(self.updateTurnProperties)
        self.turn_clip_input.valueChanged.connect(self.updateTurnProperties)

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
        self.snap_intersection_button = QPushButton("Auto-align This Waypoint")
        self.snap_intersection_button.setToolTip("Align robot centre so the light sensor is on the selected LEFT/RIGHT edge of the incoming black track at the nearby junction")
        intersection_layout.addRow(self.snap_intersection_button)
        self.snap_intersection_button.clicked.connect(self.autoAlignSelectedWaypoint)
        self.intersection_count_input.valueChanged.connect(self.updateIntersectionProperties)
        self.intersection_direction_input.currentTextChanged.connect(self.updateIntersectionProperties)

        self.movement_group = QGroupBox("Segment Movement")
        self.movement_group.setToolTip("Line Follow uses the imported PDF's black track, a simulated edge sensor, and wheel-speed PD/PID corrections.")
        movement_layout = QFormLayout()
        self.movement_type_input = QComboBox()
        self.movement_type_input.addItems(["Drive Straight", "Line Follow (PID)"])
        self.reverse_input = QCheckBox("Reverse (drive backward without turning 180°)")
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
        movement_layout.addRow(self.reverse_input)
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
        self.reverse_input.toggled.connect(self.updateMovementProperties)
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

        self.live_debug_group = QGroupBox("Live Simulation Debugger")
        live_debug_layout = QFormLayout()
        self.debug_fields = {}
        for key, label in (("reflection", "Sensor reflection:"), ("target", "Target reflection:"), ("error", "PD/PID error:"), ("left_dps", "Left motor:"), ("right_dps", "Right motor:"), ("intersections", "Intersections:"), ("heading", "Robot heading:"), ("phase", "Movement phase:"), ("elapsed", "Simulated time:"), ("status", "Status:")):
            output = QLabel("—")
            output.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            output.setWordWrap(True)
            live_debug_layout.addRow(label, output)
            self.debug_fields[key] = output
        self.debug_speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.debug_speed_slider.setRange(10, 400)
        self.debug_speed_slider.setSingleStep(10)
        self.debug_speed_slider.setValue(100)
        self.debug_speed_label = QLabel("1.00×")
        self.debug_speed_slider.valueChanged.connect(self.setDebugPlaybackSpeed)
        live_debug_layout.addRow("Playback speed:", self.debug_speed_slider)
        live_debug_layout.addRow("Speed multiplier:", self.debug_speed_label)
        self.debug_step_button = QPushButton("Step +20 ms")
        self.debug_step_button.setToolTip("Advance exactly 20 ms of simulated motion and remain paused")
        self.debug_step_button.clicked.connect(self.stepSimulation)
        live_debug_layout.addRow(self.debug_step_button)
        self.debug_trail_toggle = QCheckBox("Show actual robot trail")
        self.debug_trail_toggle.setChecked(True)
        self.debug_trail_toggle.toggled.connect(self.canvas.setTrailVisible)
        live_debug_layout.addRow(self.debug_trail_toggle)
        self.debug_clear_trail_button = QPushButton("Clear Trail")
        self.debug_clear_trail_button.clicked.connect(self.canvas.clearSimulationTrail)
        live_debug_layout.addRow(self.debug_clear_trail_button)
        self.live_debug_group.setLayout(live_debug_layout)
        properties_layout.addWidget(self.live_debug_group)
        self.canvas.debugUpdated.connect(self.updateSimulationDebug)
        self.canvas.publishDebug()

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

        export_action = QAction("Export Pybricks Route...", self)
        export_action.triggered.connect(self.exportPybricksRoute)
        file_menu.addAction(export_action)
        import_code_action = QAction("Import Pybricks Route...", self)
        import_code_action.triggered.connect(self.importPybricksRoute)
        file_menu.addAction(import_code_action)

        toolbar = QToolBar("Main Toolbar", self)
        self.addToolBar(toolbar)

        toolbar.addAction(import_action)
        toolbar.addAction(export_action)
        toolbar.addAction(import_code_action)
        builder_action = QAction("Robot Builder…", self)
        builder_action.triggered.connect(lambda: RobotBuilder(self.canvas, self).exec())
        toolbar.addAction(builder_action)
        self.menuBar().addMenu("Robot").addAction(builder_action)
        auto_snap_action = QAction("Auto-align Intersection Waypoints", self)
        auto_snap_action.triggered.connect(self.canvas.snapAllIntersectionWaypoints)
        toolbar.addAction(auto_snap_action)
        auto_snap_toggle = QAction("Auto-snap Intersection Waypoints on Drag", self)
        auto_snap_toggle.setCheckable(True)
        auto_snap_toggle.setChecked(True)
        auto_snap_toggle.toggled.connect(lambda enabled: setattr(self.canvas, "auto_snap_intersections", enabled))
        view_menu_intersections = self.menuBar().addMenu("Intersections")
        view_menu_intersections.addAction(auto_snap_action)
        view_menu_intersections.addAction(auto_snap_toggle)

        track_debug_action = QAction("Show Detected Track", self)
        track_debug_action.setCheckable(True)
        track_debug_action.setToolTip("Highlight pixels treated as black by the line tracker (cyan). Read-only; does not affect editing or sensor readings.")
        track_debug_action.toggled.connect(self.canvas.setTrackDebugVisible)
        toolbar.addAction(track_debug_action)
        view_menu = self.menuBar().addMenu("View")
        view_menu.addAction(self.properties_dock.toggleViewAction())
        view_menu.addAction(track_debug_action)
        grid_action = QAction("Show 100 mm Coordinate Grid", self)
        grid_action.setCheckable(True)
        grid_action.setChecked(True)
        grid_action.toggled.connect(lambda visible: (setattr(self.canvas, "show_coordinate_grid", visible), self.canvas.viewport().update()))
        view_menu.addAction(grid_action)
        self.canvas.navigation_overlay = NavigationOverlay(self, view_menu)

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

        demo_menu=self.menuBar().addMenu("Examples")
        demo_action=QAction("Load 2026 Senior — Mosaic Masters",self)
        demo_action.triggered.connect(lambda: self.loadSenior2026Demo())
        demo_menu.addAction(demo_action)
        self.setupFieldEditor(view_menu)
        self.undo_history=[self.canvas.editSnapshot()]; self.redo_history=[]; self.restoring_history=False
        edit_menu=self.menuBar().addMenu("Edit")
        self.undo_action=QAction("Undo",self); self.undo_action.setShortcuts(QKeySequence.StandardKey.Undo); self.undo_action.triggered.connect(self.undoEdit); edit_menu.addAction(self.undo_action)
        self.redo_action=QAction("Redo",self); self.redo_action.setShortcuts(QKeySequence.StandardKey.Redo); self.redo_action.triggered.connect(self.redoEdit); edit_menu.addAction(self.redo_action)
        self.history_timer=QTimer(self); self.history_timer.setInterval(120); self.history_timer.timeout.connect(self.captureEdit); self.history_timer.start()

    def loadSenior2026Demo(self, confirm=True):
        if confirm and QMessageBox.question(self,"Load example","Replace the current field, robot and route with the 2026 Senior test fixture?") != QMessageBox.StandardButton.Yes: return
        c=self.canvas
        path=Path(__file__).resolve().with_name("WRO-2026-GameMat-Senior-Printing-File(3).pdf")
        if not path.exists():
            selected,_=QFileDialog.getOpenFileName(self,"Locate 2026 Senior mat PDF","","PDF Files (*.pdf)")
            if not selected: return
            path=Path(selected)
        if not c.importField(str(path)):
            QMessageBox.warning(self,"Example","Could not load the game mat."); return
        self.restoring_history=True
        try:
            scale=c.pixels_per_mm
            state=c.editSnapshot()
            state['robot'].update(width=250.0,length=250.0,wheel_diameter=62.4,axle_track=190.0,initial_heading=0.0,sensor_x_mm=80.0,sensor_y_mm=0.0,sensor_diameter_mm=18.0,parts=senior2026Parts())
            state.update(pose=(430.0,595.5,0.0),vertices=[(-125*scale,-125*scale),(125*scale,-125*scale),(125*scale,125*scale),(-125*scale,125*scale)],waypoints=[],segments=[],next_id=0,objects=senior2026Objects(scale))
            c.restoreEditSnapshot(state)
            # Right edge of the long bottom horizontal track. Deliberately short debugging route.
            moves=[dict(type='line',distance=100.0,stop='Distance',side='right',estimated=False),dict(type='line',distance=(605-430)/scale-100,stop='Intersection',side='right',estimated=False,target_scene=QPointF(605,595.5)),dict(type='straight',distance=100.0,stop='Distance',side='right',estimated=False),dict(type='straight',distance=-50.0,stop='Distance',side='right',estimated=False),dict(type='turn',angle=-90.0)]
            c.loadRouteMovements(moves,QPointF(430,595.5),0.0)
            c.scene.clearSelection(); c.scene.setSceneRect(c.background.boundingRect()); c.fitInView(c.background,Qt.AspectRatioMode.KeepAspectRatio); c.zoom_level=c.transform().m11()
        finally:
            self.restoring_history=False
        self.undo_history=[c.editSnapshot()]; self.redo_history=[]; self.captureEdit(True)
        self.statusBar().showMessage("2026 Senior example loaded — editable geometric test fixture, not a full mission solution.",12000)

    def setupFieldEditor(self,view_menu):
        self.field_dock=QDockWidget("Field Objects",self); panel=QWidget(); layout=QVBoxLayout(panel)
        layout.addWidget(QLabel("Drag an object onto the mat, or double-click to add."))
        self.field_palette=FieldPalette(); self.field_palette.addItems(list(FIELD_TYPES)); self.field_palette.setDragEnabled(True); layout.addWidget(self.field_palette)
        self.field_palette.itemDoubleClicked.connect(lambda item: self.canvas.addFieldObject(item.text(),self.canvas.mapToScene(self.canvas.viewport().rect().center())))
        self.field_group=QGroupBox("Selected object"); form=QFormLayout(self.field_group); layout.addWidget(self.field_group)
        self.field_name=QLineEdit(); form.addRow("Name",self.field_name); self.field_name.textEdited.connect(self.writeFieldObject)
        self.field_controls={}
        for key,label,low,high in [('x','X',-10000,10000),('y','Y',-10000,10000),('length','Length',1,3000),('width','Width',1,3000),('height','Height',0,3000),('angle','Rotation',-360,360)]:
            c=QDoubleSpinBox(); c.setRange(low,high); c.setDecimals(1); c.setSuffix('°' if key=='angle' else ' mm'); form.addRow(label,c); self.field_controls[key]=c; c.valueChanged.connect(self.writeFieldObject)
        self.field_color=QPushButton("Change colour…"); self.field_color.clicked.connect(self.chooseFieldColor); form.addRow(self.field_color)
        duplicate=QPushButton("Duplicate"); duplicate.clicked.connect(self.duplicateFieldObject); form.addRow(duplicate)
        remove=QPushButton("Delete object"); remove.clicked.connect(self.deleteFieldObject); form.addRow(remove)
        layout.addStretch(); self.field_dock.setWidget(panel); self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea,self.field_dock)
        view_menu.addAction(self.field_dock.toggleViewAction()); self.field_group.setEnabled(False)
        self.field_loading=False; self.canvas.itemSelected.connect(self.readFieldObject)

    def selectedFieldObject(self):
        items=self.canvas.scene.selectedItems(); return items[0] if len(items)==1 and isinstance(items[0],FieldObject) else None

    def readFieldObject(self,item):
        if not isinstance(item,FieldObject): item=None
        self.field_group.setEnabled(item is not None)
        if item is None: return
        self.field_loading=True
        try:
            self.field_name.setText(item.data['name'])
            for k,c in self.field_controls.items(): c.setValue(item.data[k])
            self.field_color.setStyleSheet('background-color: '+item.data['color'])
        finally: self.field_loading=False

    def writeFieldObject(self,*args):
        item=self.selectedFieldObject()
        if self.field_loading or item is None: return
        item.data.update(name=self.field_name.text(),**{k:c.value() for k,c in self.field_controls.items()}); item.refresh()

    def chooseFieldColor(self):
        item=self.selectedFieldObject()
        if item is None: return
        color=QColorDialog.getColor(QColor(item.data['color']),self)
        if color.isValid(): item.data['color']=color.name(); item.refresh(); self.readFieldObject(item)

    def duplicateFieldObject(self):
        old=self.selectedFieldObject()
        if old is None:return
        d=copy.deepcopy(old.data); d['x']+=20; d['y']+=20; d['name']+=' copy'
        item=FieldObject(self.canvas,d); self.canvas.field_objects.append(item); self.canvas.scene.addItem(item); self.canvas.scene.clearSelection(); item.setSelected(True)

    def deleteFieldObject(self):
        item=self.selectedFieldObject()
        if item is not None: self.canvas.field_objects.remove(item); self.canvas.scene.removeItem(item); self.readFieldObject(None)

    def captureEdit(self,force=False):
        if self.restoring_history or (not force and (QApplication.mouseButtons()!=Qt.MouseButton.NoButton or QApplication.activeModalWidget() is not None)): return
        state=self.canvas.editSnapshot()
        if state!=self.undo_history[-1]:
            self.undo_history.append(state); self.undo_history=self.undo_history[-101:]; self.redo_history.clear()
        self.undo_action.setEnabled(True); self.redo_action.setEnabled(bool(self.redo_history))

    def undoEdit(self):
        self.captureEdit(True)
        if len(self.undo_history)<2:return
        self.redo_history.append(self.undo_history.pop()); self.applyHistory(self.undo_history[-1])

    def redoEdit(self):
        self.captureEdit(True)
        if not self.redo_history:return
        state=self.redo_history.pop(); self.undo_history.append(state); self.applyHistory(state)

    def applyHistory(self,state):
        self.restoring_history=True
        try: self.canvas.restoreEditSnapshot(state)
        finally: self.restoring_history=False
        # Use restored state as baseline (normalisation may update derived dimensions).
        self.undo_history[-1]=self.canvas.editSnapshot()
        self.undo_action.setEnabled(True); self.redo_action.setEnabled(bool(self.redo_history))

    def importPybricksRoute(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("Import Pybricks Movement Calls")
        dialog.resize(800, 620)
        layout = QVBoxLayout(dialog)
        info = QLabel("Paste minimal Pybricks movement calls, including negative distances for reversing. The robot's current starting position and heading are used. The route replaces existing waypoints only after you click Generate. Grid axes are in millimetres relative to Waypoint 0; +Y points down on screen.")
        info.setWordWrap(True)
        layout.addWidget(info)
        editor = QTextEdit(dialog)
        editor.setPlaceholderText('gyroDriveStraight(300)\nturn(90)\nsingleLineTrackByDist(450, side="left")\nturn(-90)\nsingleLineTrackTillIntersection()')
        layout.addWidget(editor)
        auto_detect = QCheckBox("Auto-detect next intersection from imported PDF (straight approaches)")
        auto_detect.setChecked(self.canvas.background_image is not None)
        auto_detect.setEnabled(self.canvas.background_image is not None)
        auto_detect.setToolTip("Search along the current heading for the next detected T/cross junction, then align to the selected track edge. Requires a loaded PDF and correctly positioned starting robot.")
        layout.addWidget(auto_detect)
        estimation_row = QHBoxLayout()
        estimation_row.addWidget(QLabel("Fallback intersection distance:"))
        estimate_input = QDoubleSpinBox(dialog)
        estimate_input.setRange(10, 5000)
        estimate_input.setDecimals(0)
        estimate_input.setValue(200)
        estimate_input.setSuffix(" mm")
        estimation_row.addWidget(estimate_input)
        layout.addLayout(estimation_row)
        note = QLabel("With auto-detection, each 'track till intersection' command searches the loaded PDF ahead of the current position and heading, then aligns the robot centre to the chosen line edge. An unresolved junction uses the fallback distance. This is geometric placement, not physical line-following; place the starting robot and heading accurately.")
        note.setWordWrap(True)
        layout.addWidget(note)
        buttons = QHBoxLayout()
        generate_button = QPushButton("Generate Route on Map")
        cancel_button = QPushButton("Cancel")
        buttons.addWidget(generate_button)
        buttons.addWidget(cancel_button)
        layout.addLayout(buttons)
        def generate():
            try:
                movements, warnings = parsePybricksRoute(editor.toPlainText(), estimate_input.value(), auto_intersections=auto_detect.isChecked())
                if auto_detect.isChecked():
                    warnings.extend(self.canvas.resolveImportedIntersections(movements))
            except ValueError as exc:
                QMessageBox.warning(dialog, "Cannot parse movement calls", str(exc))
                return
            if warnings:
                QMessageBox.information(dialog, "Approximate route sections", "\n".join(warnings))
            self.canvas.loadRouteMovements(movements)
            self.updatePreviewControls()
            dialog.accept()
        generate_button.clicked.connect(generate)
        cancel_button.clicked.connect(dialog.reject)
        dialog.exec()

    def exportPybricksRoute(self):
        code, warnings, errors = generatePybricksRoute(self.canvas.waypoints, self.canvas.segments, self.canvas.robot)
        if errors:
            QMessageBox.warning(self, "Cannot export route", "Fix these issues first:\n\n" + "\n".join("• " + error for error in errors))
            return
        dialog = QDialog(self)
        dialog.setWindowTitle("Review Pybricks Route")
        dialog.resize(920, 700)
        layout = QVBoxLayout(dialog)
        info = QLabel("Minimal movement calls for your existing competition-day Pybricks main.py. Paste them where the route should run; your existing helper functions and default speeds are used. Not a standalone program.")
        info.setWordWrap(True)
        layout.addWidget(info)
        if warnings:
            warning_label = QLabel("Warnings (not included in exported code):\n" + "\n".join("• " + warning for warning in warnings))
            warning_label.setWordWrap(True)
            layout.addWidget(warning_label)
        editor = QTextEdit(dialog)
        editor.setPlainText(code)
        editor.setReadOnly(False)
        layout.addWidget(editor)
        button_row = QHBoxLayout()
        save_button = QPushButton("Save Route .py")
        close_button = QPushButton("Close")
        button_row.addWidget(save_button)
        button_row.addWidget(close_button)
        layout.addLayout(button_row)
        def save_route():
            path, _ = QFileDialog.getSaveFileName(dialog, "Save Generated Pybricks Route", "generated_pybricks_route.py", "Python Files (*.py)")
            if not path:
                return
            try:
                compile(editor.toPlainText(), path, "exec")
                with open(path, "w", encoding="utf-8") as file:
                    file.write(editor.toPlainText())
            except (OSError, SyntaxError) as exc:
                QMessageBox.critical(dialog, "Could not save route", str(exc))
                return
            QMessageBox.information(dialog, "Route saved", "Movement calls saved. Paste them into your competition-day robot script at the desired location, below the helper definitions. This snippet is not standalone.")
        save_button.clicked.connect(save_route)
        close_button.clicked.connect(dialog.accept)
        dialog.exec()

    def updateProperties(self, item):
        if isinstance(item, FieldObject):
            item=None
        if isinstance(item, Waypoint):
            id = item.waypoint_id
            self.selected_label.setText(f"Selected Item: Waypoint {id}")
            self.distance_label.setText("Distance: —")
            self.angle_label.setText("Heading: —")
            turn = self.canvas.calculateTurn(item)
            route_turn = turn if turn is not None else 0.0
            requested_turn = item.turn_angle_deg if item.turn_override_enabled else ({"Left": -90.0, "Right": 90.0, "Straight": 0.0}.get(item.intersection_direction, route_turn))
            effective_turn = max(-item.turn_clip_deg, min(item.turn_clip_deg, requested_turn))
            self.turn_label.setText(f"Effective turn: {effective_turn:+.1f}° (route: {route_turn:+.1f}°)" if item in self.canvas.waypoints[:-1] or item.turn_override_enabled else "Turn: final waypoint (no custom angle)")
            with QSignalBlocker(self.turn_override_input), QSignalBlocker(self.turn_angle_input), QSignalBlocker(self.turn_clip_input):
                self.turn_override_input.setChecked(item.turn_override_enabled)
                self.turn_angle_input.setValue(item.turn_angle_deg if item.turn_override_enabled else (turn or 0.0))
                self.turn_clip_input.setValue(item.turn_clip_deg)
            self.turn_angle_input.setEnabled(True)
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
            with QSignalBlocker(self.reverse_input), QSignalBlocker(self.movement_type_input), QSignalBlocker(self.speed_input), QSignalBlocker(self.turn_speed_input), QSignalBlocker(self.line_side_input), QSignalBlocker(self.line_stop_input), QSignalBlocker(self.line_target_input), QSignalBlocker(self.kp_input), QSignalBlocker(self.ki_input), QSignalBlocker(self.kd_input):
                self.movement_type_input.setCurrentText(item.movement_type)
                self.reverse_input.setChecked(item.reverse)
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
        self.turn_group.setEnabled(isinstance(item, Waypoint))
        self.snap_intersection_button.setEnabled(isinstance(item, Waypoint) and self.canvas.background_image is not None)
        self.intersection_group.setEnabled(isinstance(item, Waypoint) and any(segment.end_waypoint is item and segment.movement_type == "Line Follow (PID)" and segment.line_stop == "Intersection" for segment in item.connected_segments))
        line_follow = isinstance(item, PathSegment) and item.movement_type == "Line Follow (PID)"
        for control in (self.line_side_input, self.line_stop_input, self.line_target_input, self.kp_input, self.ki_input, self.kd_input):
            control.setEnabled(line_follow)
        self.robot_add_vertex_button.setEnabled(isinstance(item, RobotPolygon) and self.canvas.robot_item.edit_mode)
        self.robot_remove_vertex_button.setEnabled(isinstance(item, RobotPolygon) and self.canvas.robot_item.edit_mode and self.canvas.robot_item.selected_vertex is not None and len(self.canvas.robot_item.vertices) > 3)

    def autoAlignSelectedWaypoint(self):
        selected = self.canvas.scene.selectedItems()
        if len(selected) == 1 and isinstance(selected[0], Waypoint):
            self.canvas.snapWaypointToIntersection(selected[0])

    def updateTurnProperties(self, value=None):
        selected = self.canvas.scene.selectedItems()
        if len(selected) != 1 or not isinstance(selected[0], Waypoint):
            return
        waypoint = selected[0]
        waypoint.turn_override_enabled = self.turn_override_input.isChecked() or self.sender() is self.turn_angle_input
        waypoint.turn_angle_deg = self.turn_angle_input.value()
        waypoint.turn_clip_deg = self.turn_clip_input.value()
        if self.sender() is self.turn_angle_input and not self.turn_override_input.isChecked():
            with QSignalBlocker(self.turn_override_input):
                self.turn_override_input.setChecked(True)
            waypoint.turn_override_enabled = True
        self.turn_angle_input.setEnabled(True)
        route_turn = self.canvas.calculateTurn(waypoint) or 0.0
        requested = waypoint.turn_angle_deg if waypoint.turn_override_enabled else ({"Left": -90.0, "Right": 90.0, "Straight": 0.0}.get(waypoint.intersection_direction, route_turn))
        applied = max(-waypoint.turn_clip_deg, min(waypoint.turn_clip_deg, requested))
        self.turn_label.setText(f"Effective next turn: {applied:+.1f}° (route: {route_turn:+.1f}°)")
        if self.canvas.preview_playing:
            self.canvas.pauseRobotPreview()
            self.canvas.preview_paused = False
        self.canvas.refreshRobotPreview()

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
        segment.reverse = self.reverse_input.isChecked()
        segment.speed_mm_s = self.speed_input.value()
        segment.turn_speed_deg_s = self.turn_speed_input.value()
        segment.line_side = self.line_side_input.currentText()
        segment.line_stop = self.line_stop_input.currentText()
        segment.line_target = self.line_target_input.value()
        segment.kp = self.kp_input.value()
        segment.ki = self.ki_input.value()
        segment.kd = self.kd_input.value()
        segment.updateMovementAppearance()
        if segment.movement_type == "Line Follow (PID)" and segment.line_stop == "Intersection" and self.canvas.auto_snap_intersections:
            self.canvas.snapWaypointToIntersection(segment.end_waypoint, quiet=True)
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

    def setDebugPlaybackSpeed(self, value):
        self.canvas.preview_speed_multiplier = value / 100.0
        self.debug_speed_label.setText(f"{self.canvas.preview_speed_multiplier:.2f}×")
        self.canvas.publishDebug()

    def stepSimulation(self):
        if self.canvas.preview_playing:
            self.canvas.pauseRobotPreview()
        if not self.preview_button.isChecked():
            self.preview_button.setChecked(True)
        self.canvas.singleStepPreview()
        self.preview_play_button.setEnabled(not self.canvas.preview_playing)
        self.preview_pause_button.setEnabled(False)

    def updateSimulationDebug(self, data):
        def format_number(value, suffix=""):
            return "—" if value is None else f"{value:.1f}{suffix}"
        self.debug_fields["reflection"].setText(format_number(data["reflection"], "%"))
        self.debug_fields["target"].setText(format_number(data["target"], "%"))
        self.debug_fields["error"].setText(format_number(data["error"]))
        self.debug_fields["left_dps"].setText(format_number(data["left_dps"], " °/s"))
        self.debug_fields["right_dps"].setText(format_number(data["right_dps"], " °/s"))
        self.debug_fields["intersections"].setText(f"{data['intersections']} / {data['intersection_target']}" if data["intersection_target"] is not None else str(data["intersections"]))
        self.debug_fields["heading"].setText(format_number(data["heading"], "°"))
        self.debug_fields["phase"].setText(str(data["phase"]))
        self.debug_fields["elapsed"].setText(f"{data['elapsed']:.2f} s")
        self.debug_fields["status"].setText(data["status"])

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
    demo = "--demo" in sys.argv
    app = QApplication([arg for arg in sys.argv if arg != "--demo"])
    window = MainWindow()
    window.show()
    if demo: QTimer.singleShot(0, lambda: window.loadSenior2026Demo(confirm=False))
    sys.exit(app.exec())
