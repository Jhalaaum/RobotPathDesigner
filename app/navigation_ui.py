"""Thin Qt adapter; overlay is parented to the mat and never edits the route."""
from PySide6.QtGui import QAction, QImage, QPainterPath, QPen, QColor, QBrush
from PySide6.QtWidgets import QGraphicsPathItem, QGraphicsEllipseItem, QGraphicsSimpleTextItem, QMessageBox
from PySide6.QtCore import Qt

class NavigationOverlay:
    def __init__(self, window, menu):
        self.window=window; self.canvas=window.canvas; self.items=[]; self.graph=None
        self.action=QAction('Show Navigation Graph',window); self.action.setCheckable(True)
        self.action.toggled.connect(self.toggle); menu.addAction(self.action)
        rebuild=QAction('Rebuild Navigation Graph',window); rebuild.triggered.connect(self.rebuild); menu.addAction(rebuild)
    def clear(self):
        for item in self.items:
            if item.scene() is not None: item.scene().removeItem(item)
        self.items=[]; self.graph=None
    def field_changed(self):
        self.clear()
        if self.action.isChecked(): self.build()
    def toggle(self, enabled):
        if enabled and self.graph is None: self.build()
        for item in self.items: item.setVisible(enabled)
    def rebuild(self):
        self.clear()
        if self.action.isChecked(): self.build()
        else: self.action.setChecked(True)
    def build(self):
        c=self.canvas
        if c.background_image is None:
            self.window.statusBar().showMessage('Import a game mat first.',5000); return
        try:
            import numpy as np
            from navigation import extract_graph
            img=c.background_image.convertToFormat(QImage.Format.Format_RGB888)
            rgb=np.frombuffer(img.constBits(),dtype=np.uint8).reshape(img.height(),img.bytesPerLine())[:,:img.width()*3].reshape(img.height(),img.width(),3)
            self.graph=extract_graph(rgb,c.pixels_per_mm)
            def register(item,z):
                item.setZValue(z); item.setAcceptedMouseButtons(Qt.MouseButton.NoButton); self.items.append(item)
            pen=QPen(QColor('#00cfff')); pen.setWidthF(2.5); pen.setCosmetic(True)
            path=QPainterPath()
            for e in self.graph.edges:
                path.moveTo(*e.points[0])
                for p in e.points[1:]: path.lineTo(*p)
            item=QGraphicsPathItem(path,c.background); item.setPen(pen); register(item,1)
            for n in self.graph.nodes.values():
                item=QGraphicsEllipseItem(n.x-4,n.y-4,8,8,c.background); item.setPen(QPen(QColor('#713400'))); item.setBrush(QBrush(QColor('#ff9b35'))); register(item,2)
                label=QGraphicsSimpleTextItem(str(n.id),c.background); label.setPos(n.x+5,n.y-12); label.setBrush(QBrush(QColor('#bf4500'))); register(label,2)
            self.window.statusBar().showMessage(f'Navigation graph: {len(self.graph.nodes)} nodes, {len(self.graph.edges)} edges (candidate tracks).',10000)
        except Exception as exc:
            self.clear(); self.action.setChecked(False); QMessageBox.warning(self.window,'Navigation graph',str(exc))
