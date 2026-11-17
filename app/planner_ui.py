"""Map-first UI for the experimental hybrid navigator. Drop-in replacement."""
from PySide6.QtCore import Qt, Signal, QTimer, QPointF
from PySide6.QtGui import QAction, QColor, QPen, QPixmap, QBrush, QPainter, QKeySequence, QPolygonF, QImage
from PySide6.QtWidgets import (QApplication, QDialog, QVBoxLayout, QHBoxLayout, QLabel,
    QDoubleSpinBox, QPushButton, QComboBox, QPlainTextEdit, QGraphicsScene,
    QGraphicsView, QMessageBox, QSplitter, QWidget, QTableWidget, QTableWidgetItem,
    QHeaderView, QAbstractItemView, QCheckBox, QGroupBox, QGraphicsItem)
from actions import Drive, LineTrack, TrackToIntersection, Program, to_pybricks
from planner import (Pose, generate_navigation_alternatives, calculate_end_pose,
                     evaluate_candidate, estimate_execution_time)
import math
import numpy as np
from collision import CollisionChecker, Obstacle, transform_polygon
from sensor_geometry import raster_edge_validator


class DestinationView(QGraphicsView):
    destinationClicked = Signal(float, float)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            point = self.mapToScene(event.position().toPoint())
            self.destinationClicked.emit(point.x(), point.y())
            event.accept()
            return
        super().mousePressEvent(event)

    def wheelEvent(self, event):
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        if 0.05 < self.transform().m11() * factor < 12:
            self.scale(factor, factor)
        event.accept()


def install_planner(window):
    menu = window.menuBar().addMenu('Planner')
    action = QAction('Plan route…', window)
    action.setShortcut(QKeySequence('Ctrl+Shift+P'))
    action.triggered.connect(lambda: open_planner(window))
    menu.addAction(action)


def open_planner(window):
    canvas = window.canvas
    overlay = canvas.navigation_overlay
    if canvas.background_image is None:
        QMessageBox.information(window, 'Navigation planner', 'Import a game mat first.')
        return
    if overlay.graph is None:
        overlay.build()
        for item in overlay.items:
            item.setVisible(overlay.action.isChecked())
    graph = overlay.graph
    if graph is None:
        return
    origin = canvas.waypoints[0].scenePos() if canvas.waypoints else canvas.robot_item.scenePos()
    local = canvas.background.mapFromScene(origin)
    start = Pose(local.x(), local.y(), float(canvas.robot.initial_heading))
    scale = graph.pixels_per_mm
    width, height = canvas.background_image.width(), canvas.background_image.height()
    footprint = [(p.x()/canvas.robot_item.pixels_per_mm,p.y()/canvas.robot_item.pixels_per_mm) for p in canvas.robot_item.vertices]
    obstacle_items = []
    for item in canvas.field_objects:
        rect=item.rect()
        points=[rect.topLeft(),rect.topRight(),rect.bottomRight(),rect.bottomLeft()]
        pts=[canvas.background.mapFromScene(item.mapToScene(p)) for p in points]
        obstacle_items.append((item.data["kind"],Obstacle(item.data["name"],tuple((p.x(),p.y()) for p in pts))))
    img=canvas.background_image.convertToFormat(QImage.Format.Format_RGB888)
    rgb=np.frombuffer(img.constBits(),dtype=np.uint8).reshape(img.height(),img.bytesPerLine())[:,:img.width()*3].reshape(img.height(),img.width(),3)
    validator=raster_edge_validator(rgb,scale,canvas.robot)
    dialog = QDialog(window)
    dialog.setWindowTitle('Route planner')
    dialog.resize(1180, 760)
    screen = window.screen().availableGeometry()
    dialog.resize(min(1180, int(screen.width()*0.94)), min(760, int(screen.height()*0.9)))
    layout = QVBoxLayout(dialog)
    heading = QLabel('Plan your next route'); heading.setStyleSheet('font-size: 21px; font-weight: 600;')
    layout.addWidget(heading)
    layout.addWidget(QLabel('1  Pick a destination on the mat     →     2  Compare routes     →     3  Apply and edit'))
    split = QSplitter(Qt.Orientation.Horizontal); layout.addWidget(split, 1)
    left = QWidget(); ll = QVBoxLayout(left); ll.setContentsMargins(0,0,0,0)
    right = QWidget(); rl = QVBoxLayout(right); rl.setContentsMargins(0,0,0,0)
    split.addWidget(left); split.addWidget(right); split.setStretchFactor(0, 3); split.setStretchFactor(1, 2)
    split.setSizes([700,440])
    tools = QHBoxLayout(); tools.addWidget(QLabel('Click anywhere on the mat to set destination.'))
    tools.addStretch(); fit = QPushButton('Fit map'); tools.addWidget(fit); ll.addLayout(tools)
    scene = QGraphicsScene(dialog)
    view = DestinationView(scene); view.setRenderHint(QPainter.RenderHint.Antialiasing)
    view.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
    ll.addWidget(view, 1)
    legend = QLabel('● Start     ◆ Destination     Blue: drive     Green: chassis tracking     Gold dots: sensor path     ★: intersection stop')
    legend.setWordWrap(True); ll.addWidget(legend)
    coords = QGroupBox('Destination relative to start (mm)'); cl = QHBoxLayout(coords)
    x = QDoubleSpinBox(); y = QDoubleSpinBox()
    for spin, lo, hi in [(x,-start.x/scale,(width-1-start.x)/scale),(y,-start.y/scale,(height-1-start.y)/scale)]:
        spin.setDecimals(2); spin.setRange(lo,hi); spin.setSingleStep(10); spin.setSuffix(' mm'); spin.setKeyboardTracking(False)
    cl.addWidget(QLabel('X →')); cl.addWidget(x); cl.addWidget(QLabel('Y ↓')); cl.addWidget(y)
    ll.addWidget(coords)
    # The demo's existing debug route ends near a barrier. Start the planner
    # at a known clear demonstration target; clicking the mat still overrides it.
    is_demo=(abs(start.x-430.0)<1 and abs(start.y-595.5)<1 and
             any(o.name == "Mosaic frame (3 x 4)" for _,o in obstacle_items))
    if is_demo:
        x.setValue(0); y.setValue((350.0-start.y)/scale)
    elif len(canvas.waypoints)>1:
        end = canvas.background.mapFromScene(canvas.waypoints[-1].scenePos())
        x.setValue((end.x()-start.x)/scale); y.setValue((end.y()-start.y)/scale)
    else:
        x.setValue(300)
    options = QHBoxLayout(); auto = QCheckBox('Auto-plan'); auto.setChecked(True); options.addWidget(auto)
    budget = QComboBox(); budget.addItem('Quick search',30); budget.addItem('Standard search',60); budget.addItem('Deeper search',180); budget.setCurrentIndex(1)
    options.addWidget(budget); generate = QPushButton('Generate routes'); options.addWidget(generate); rl.addLayout(options)
    physical = QGroupBox("Physical constraints — draft defaults"); pl = QVBoxLayout(physical)
    check = QCheckBox("Check obstacles and mat boundary"); check.setChecked(True); pl.addWidget(check)
    movable = QCheckBox("Include movable / mission objects"); movable.setToolTip("Off by default: demo mission-object locations are approximate. Enable for fixed, verified placements."); movable.setChecked(False); pl.addWidget(movable)
    settings = QHBoxLayout(); settings.addWidget(QLabel("Clearance")); margin=QDoubleSpinBox(); margin.setRange(0,50); margin.setValue(3); margin.setSuffix(" mm"); settings.addWidget(margin)
    settings.addWidget(QLabel("Line width")); line_width=QDoubleSpinBox(); line_width.setRange(5,65); line_width.setValue(20); line_width.setSuffix(" mm"); settings.addWidget(line_width); pl.addLayout(settings)
    raster = QCheckBox("Verify tracking edge against mat image"); raster.setChecked(True); raster.setToolTip("Check the predicted sensor path for a dark/light edge. Adjust line width if no tracking routes appear."); pl.addWidget(raster); rl.addWidget(physical)
    preference = QCheckBox('Prefer useful line tracking (when a reasonable alternative)')
    preference.setChecked(True)
    preference.setToolTip('A modest, bounded ranking bonus for meaningful tracking sections. Short or long-detour routes receive no special advantage.')
    rl.addWidget(preference)
    filters = QHBoxLayout(); filters.addWidget(QLabel('Show:')); filter_box = QComboBox(); filter_box.addItems(['All routes','Non-tracking only','Hybrid only']); filters.addWidget(filter_box,1); rl.addLayout(filters)
    status = QLabel('Pick a destination to begin.'); status.setWordWrap(True); rl.addWidget(status)
    table = QTableWidget(0,5); table.setHorizontalHeaderLabels(['Route','Time (s)','Track (mm)','Actions','Cost'])
    table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers); table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows); table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
    table.verticalHeader().hide(); table.horizontalHeader().setSectionResizeMode(0,QHeaderView.ResizeMode.Stretch)
    for col in range(1,5): table.horizontalHeader().setSectionResizeMode(col,QHeaderView.ResizeMode.ResizeToContents)
    table.setAlternatingRowColors(True); rl.addWidget(table, 2)
    summary = QLabel('Select a route to preview.'); summary.setWordWrap(True); rl.addWidget(summary)
    codebar = QHBoxLayout(); codebar.addWidget(QLabel('Pybricks preview')); codebar.addStretch(); copy_btn = QPushButton('Copy code'); copy_btn.setEnabled(False); codebar.addWidget(copy_btn); rl.addLayout(codebar)
    editor = QPlainTextEdit(); editor.setReadOnly(True); editor.setStyleSheet('font-family: monospace;'); rl.addWidget(editor,1)
    warning = QLabel('Experimental: nominal footprint and sensor edge checks; no physical feedback, articulated attachments, or uncertainty model. Review before physical use.'); warning.setWordWrap(True); layout.addWidget(warning)
    controls = QHBoxLayout(); controls.addStretch(); close = QPushButton('Close'); apply = QPushButton('Apply selected route…'); apply.setEnabled(False); apply.setDefault(False); controls.addWidget(close); controls.addWidget(apply); layout.addLayout(controls)
    choices = []; visible = []; destination = [start.x,start.y]; busy = False
    pixmap = QPixmap.fromImage(canvas.background_image)
    timer = QTimer(dialog); timer.setSingleShot(True); timer.setInterval(350)

    def fit_map():
        view.fitInView(scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    def redraw(actions=()):
        scene.clear(); image = scene.addPixmap(pixmap); scene.setSceneRect(image.boundingRect())
        for kind, obstacle in obstacle_items:
            if kind in ("Target zone","Terrain region","Grid / matrix","Colour marker"): continue
            if not movable.isChecked() and kind not in ("Static obstacle","Barrier"): continue
            poly=QPolygonF([QPointF(*p) for p in obstacle.polygon]); pen=QPen(QColor("#ba573f")); pen.setCosmetic(True)
            scene.addPolygon(poly,pen,QBrush(QColor(200,80,50,65)))
        pose = start
        for action in actions:
            nxt = calculate_end_pose(pose,(action,),scale)
            if isinstance(action, TrackToIntersection):
                marker = scene.addSimpleText('★')
                marker.setBrush(QBrush(QColor('#f0a500')))
                marker.setPos(nxt.x-7, nxt.y-17)
                marker.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
            if isinstance(action,LineTrack):
                a=math.radians(pose.heading); f=canvas.robot.sensor_x_mm*scale; r=canvas.robot.sensor_y_mm*scale
                dx=f*math.cos(a)-r*math.sin(a); dy=f*math.sin(a)+r*math.cos(a)
                sensor_pen=QPen(QColor("#d89527")); sensor_pen.setWidthF(2); sensor_pen.setCosmetic(True); sensor_pen.setStyle(Qt.PenStyle.DotLine)
                scene.addLine(pose.x+dx,pose.y+dy,nxt.x+dx,nxt.y+dy,sensor_pen)
            if isinstance(action,(Drive,LineTrack)):
                pen = QPen(QColor('#008f55' if isinstance(action,LineTrack) else '#1676e4')); pen.setWidthF(3.5); pen.setCosmetic(True)
                if isinstance(action,Drive) and action.distance < 0: pen.setStyle(Qt.PenStyle.DashLine)
                scene.addLine(pose.x,pose.y,nxt.x,nxt.y,pen)
            pose = nxt
        for marker_pose in (start,pose) if actions else (start,):
            poly=QPolygonF([QPointF(*p) for p in transform_polygon(footprint,marker_pose,scale)])
            pen=QPen(QColor("#267d59")); pen.setCosmetic(True); scene.addPolygon(poly,pen,QBrush(QColor(55,155,100,45)))
        for px,py,color,name in [(start.x,start.y,'#1676e4','START'),(*destination,'#eb6037','TARGET')]:
            pen = QPen(QColor('white')); pen.setWidthF(2); pen.setCosmetic(True)
            item = scene.addEllipse(px-7,py-7,14,14,pen,QBrush(QColor(color)))
            label = scene.addSimpleText(name); label.setBrush(QBrush(QColor(color))); label.setPos(px+9,py-14)
            label.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        # Show arrival orientation, not just the final point.
        if actions:
            a = math.radians(pose.heading); pen = QPen(QColor('#eb6037')); pen.setWidthF(3); pen.setCosmetic(True)
            scene.addLine(pose.x,pose.y,pose.x+22*math.cos(a),pose.y+22*math.sin(a),pen)

    def display(*_):
        idx = table.currentRow()
        if not 0 <= idx < len(visible):
            redraw(); editor.clear(); apply.setEnabled(False); copy_btn.setEnabled(False); summary.setText('No route selected.'); return
        cost, seconds, actions, track, label = visible[idx]
        redraw(actions); editor.setPlainText(to_pybricks(Program(actions)))
        end = calculate_end_pose(start,actions,scale)
        summary.setText(f'{label} · {seconds:.2f} s estimated · {track:.0f} mm tracking · arrival heading {end.heading:.1f}°' + (' · junction stop: predicted position only' if any(isinstance(a, TrackToIntersection) for a in actions) else ''))
        apply.setEnabled(bool(actions)); copy_btn.setEnabled(True)

    def populate(*_):
        mode = filter_box.currentText()
        filtered = [c for c in choices if mode == 'All routes' or (mode == 'Hybrid only') == (c[3] > 0)]
        # Show the best strategy representatives first, rather than many almost
        # identical paths that differ only by a few millimetres or milliseconds.
        leaders, shown, per_label = [], [], {}
        for c in filtered:
            if c[4] not in per_label:
                leaders.append(c)
                per_label[c[4]] = 1
        for c in filtered:
            if c in leaders or per_label.get(c[4],0) >= 3:
                continue
            peers=[p for p in shown+leaders if p[4]==c[4]]
            if any(abs(p[1]-c[1])<0.45 and abs(p[3]-c[3])<85
                   and abs(len(p[2])-len(c[2]))<3 for p in peers):
                continue
            shown.append(c)
            per_label[c[4]] = per_label.get(c[4],0)+1
            if len(shown)>=18: break
        visible[:] = leaders + shown
        table.blockSignals(True); table.setRowCount(len(visible))
        for row, (cost, seconds, actions, track, label) in enumerate(visible):
            for col, value in enumerate([label, f'{seconds:.2f}',f'{track:.0f}',str(len(actions)),f'{cost:.2f}']):
                table.setItem(row,col,QTableWidgetItem(value))
        table.clearSelection(); table.setCurrentCell(0,0) if visible else table.setCurrentCell(-1,-1)
        table.blockSignals(False); display()

    def run():
        nonlocal busy
        if busy: return
        timer.stop(); busy = True; generate.setEnabled(False); apply.setEnabled(False)
        status.setText('Searching route alternatives…'); QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        try:
            destination[:] = [start.x+x.value()*scale,start.y+y.value()*scale]
            blocked=[o for kind,o in obstacle_items if kind in ("Static obstacle","Barrier") or (movable.isChecked() and kind not in ("Target zone","Terrain region","Grid / matrix","Colour marker"))]
            checker=CollisionChecker(footprint,blocked,scale,bounds=(0,0,width-1,height-1),margin_mm=margin.value()) if check.isChecked() else None
            if checker is not None:
                initial_hit=checker.check_pose(start)
                if initial_hit:
                    choices.clear(); populate(); status.setText(f"Starting footprint violates clearance: {initial_hit.obstacle}. Adjust start, objects or clearance."); return
            routes = generate_navigation_alternatives(
                start,destination,canvas.robot,scale,graph=graph,
                collision_checker=checker,max_pairs=budget.currentData(),
                prefer_line_tracking=preference.isChecked(),
                sensor_aware=True,line_width_mm=line_width.value(),
                edge_validator=validator if raster.isChecked() else None)
            choices.clear()
            for label, actions in routes:
                track=sum(abs(a.distance) for a in actions if isinstance(a,LineTrack))
                choices.append((evaluate_candidate(start,actions,canvas.robot,scale,
                                               prefer_line_tracking=preference.isChecked()),
                                estimate_execution_time(actions,canvas.robot),actions,track,label))
            counts={name:sum(1 for c in choices if c[4]==name) for name in ('Forward','Reverse','L-shaped','Detour','Line-augmented','Intersection track','Junction corner','Opportunistic track','Hybrid','Shortcut')}
            status.setText((" · ".join(f"{n} {name.lower()}" for name,n in counts.items() if n)
                            + ". Ranked by draft navigation cost (uncertainty not yet modelled)."
                            + (" Collision checks OFF." if checker is None else ""))
                           if choices else 'No feasible route found. Try a different destination, inspect field objects, or adjust the clearance. The detour search is still limited.')
            populate()
        except Exception as exc:
            choices.clear(); populate(); status.setText(f'Planning failed: {exc}')
        finally:
            busy=False; generate.setEnabled(True); QApplication.restoreOverrideCursor()

    def invalidate(*_):
        destination[:] = [start.x+x.value()*scale,start.y+y.value()*scale]
        choices.clear(); populate(); status.setText('Destination or search settings changed.')
        if auto.isChecked(): timer.start()

    def choose_destination(px,py):
        if 0<=px<width and 0<=py<height:
            x.blockSignals(True); y.blockSignals(True)
            x.setValue((px-start.x)/scale); y.setValue((py-start.y)/scale)
            x.blockSignals(False); y.blockSignals(False); invalidate()

    def commit():
        idx=table.currentRow()
        if not 0<=idx<len(visible) or not visible[idx][2]: return
        if QMessageBox.question(dialog,'Replace editable route?', 'Apply this experimental route and replace the existing waypoints? You can undo the change in the main window.') != QMessageBox.StandardButton.Yes: return
        try:
            window.captureEdit(True); canvas.loadActionProgram(Program(visible[idx][2]),origin_scene=origin,initial_heading=start.heading)
            window.captureEdit(True); window.updatePreviewControls(); dialog.accept()
        except Exception as exc:
            QMessageBox.warning(dialog,'Cannot apply route',str(exc))

    view.destinationClicked.connect(choose_destination); fit.clicked.connect(fit_map)
    x.valueChanged.connect(invalidate); y.valueChanged.connect(invalidate); budget.currentIndexChanged.connect(invalidate)
    auto.toggled.connect(lambda on: timer.start() if on and not choices else timer.stop())
    filter_box.currentIndexChanged.connect(populate); table.itemSelectionChanged.connect(display)
    for control in (check,movable,raster,preference): control.toggled.connect(invalidate)
    for control in (margin,line_width): control.valueChanged.connect(invalidate)
    generate.clicked.connect(run); timer.timeout.connect(run); apply.clicked.connect(commit); close.clicked.connect(dialog.reject)
    copy_btn.clicked.connect(lambda: QApplication.clipboard().setText(editor.toPlainText()))
    invalidate(); QTimer.singleShot(0,fit_map); dialog.exec()
