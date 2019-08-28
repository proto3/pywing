from PyQt5 import Qt, QtCore, QtGui
import os, sys

from pathgenerator import PathGenerator
from path import Path
from airfoilloader import AirfoilLoader
from dxfloader import DXFLoader
from svgloader import SVGLoader

import numpy as np
import datetime
import pyqtgraph as pg

class PathManager(QtCore.QObject):
    gen_update = QtCore.pyqtSignal()
    sync_update = QtCore.pyqtSignal()
    reset = QtCore.pyqtSignal()

    def __init__(self, color):
        super().__init__()
        self.path = Path()
        self.gen = self.shift_gen = self.sync_gen = PathGenerator()
        self.raw_path = np.array([[],[]])

        self.name = ''
        self.color = color
        self.loaded = False
        self.shift = 0.0

    def export_tuple(self):
        return self.path, self.gen, self.name, self.color, self.loaded, self.shift

    def import_tuple(self, tuple):
        self.path, self.gen, self.name, self.color, self.loaded, self.shift = tuple
        self.reset.emit()
        self.sync_update.emit()

    def scale(self, s):
        #TODO PathGenerator should adapt arc resolution to scale
        self.path.scale(s)
        self.gen_update.emit()

    def rotate(self, r):
        self.path.rotate(r)
        self.gen_update.emit()

    def translate_x(self, t):
        self.path.translate_x(t)
        self.gen_update.emit()

    def translate_y(self, t):
        self.path.translate_y(t)
        self.gen_update.emit()

    def set_kerf_width(self, k):
        self.path.set_kerf_width(k)
        self.gen_update.emit()

    def set_lead_size(self, l):
        self.path.set_lead_size(l)
        self.gen_update.emit()

    def get_scale(self):
        return self.path.s

    def get_kerf_width(self):
        return self.path.k

    def load(self, filename):
        extension = os.path.splitext(os.path.basename(filename))[1].upper()
        try:
            if extension == '.DAT' or extension == '.COR':
                self.gen = self.shift_gen = self.sync_gen = AirfoilLoader.load(filename)
            elif extension == '.DXF':
                self.gen = self.shift_gen = self.sync_gen = DXFLoader.load(filename)
            elif extension == '.SVG':
                self.gen = self.shift_gen = self.sync_gen = SVGLoader.load(filename)
        except Exception:
            print(sys.exc_info())
            return

        self.name = os.path.basename(filename)
        self.loaded = True

        self.reset.emit()
        self.sync_update.emit()

    def synchronize(a, b):
        a.shift_gen = a.gen.rotate(a.shift)
        b.shift_gen = b.gen.rotate(b.shift)
        a.sync_gen, b.sync_gen = PathGenerator.synchronize(a.shift_gen, b.shift_gen)

    def generate(self):
        self.path.initial_path = self.sync_gen.generate()
        self.path._apply_transform()

    def close_to(self, p):
        return self.gen.close_to(p)

    def set_shift(self, shift):
        self.shift = shift
        self.sync_update.emit()

    def get_shift(self):
        return self.shift

    def reverse(self):
        self.gen.reverse()
        self.sync_update.emit()

    def add_sync_point(self, degree):
        self.gen.add_sync_point(degree)
        self.sync_update.emit()

    def remove_sync_point(self, degree):
        self.gen.remove_sync_point(degree)
        self.sync_update.emit()

class PathManagerWidget(QtGui.QWidget):
    def __init__(self, path_manager):
        super().__init__()
        self.pm = path_manager

        self.name = QtGui.QLabel(alignment=Qt.Qt.AlignCenter)
        self.update_name()

        self.load_btn = QtGui.QPushButton("Load")
        self.load_btn.clicked.connect(self.on_load)

        self.reverse_btn = QtGui.QPushButton("Reverse")
        self.reverse_btn.clicked.connect(self.on_reverse)

        self.scale_spbox = QtGui.QDoubleSpinBox()
        self.scale_spbox.setRange(1, 10000)
        self.scale_spbox.setValue(self.pm.get_scale())
        self.scale_spbox.setPrefix("S : ")
        self.scale_spbox.setSuffix("mm")
        self.scale_spbox.valueChanged.connect(self.on_scale)

        self.kerf_spbox = QtGui.QDoubleSpinBox()
        self.kerf_spbox.setRange(0, 100)
        self.kerf_spbox.setValue(self.pm.get_kerf_width())
        self.kerf_spbox.setSingleStep(0.1)
        self.kerf_spbox.setPrefix("K : ")
        self.kerf_spbox.setSuffix("mm")
        self.kerf_spbox.valueChanged.connect(self.on_kerf)

        self.shift_spbox = QtGui.QDoubleSpinBox()
        self.shift_spbox.setRange(0, 1)
        self.shift_spbox.setValue(self.pm.get_shift())
        self.shift_spbox.setSingleStep(0.01)
        self.shift_spbox.setPrefix("SH : ")
        self.shift_spbox.valueChanged.connect(self.on_shift)

        self.sync_view = SyncViewWidget(self.pm, self.pm.color)

        self.widgets = (self.name, self.load_btn, self.scale_spbox, self.kerf_spbox, self.shift_spbox, self.reverse_btn, self.sync_view)

        layout = QtGui.QVBoxLayout()
        [layout.addWidget(w) for w in self.widgets]
        layout.addStretch()
        self.setLayout(layout)

        self.pm.reset.connect(self.update)

    def on_load(self):
        filename, _ = QtGui.QFileDialog.getOpenFileName(self.load_btn.parent(), 'Open File', QtCore.QDir.homePath(), 'Airfoil, DXF, SVG (*.dat *.cor *.dxf *.DXF *.svg);; All Files (*)')
        if filename:
            self.pm.load(filename)

    def on_scale(self):
        self.pm.scale(self.scale_spbox.value())

    def on_kerf(self):
        self.pm.set_kerf_width(self.kerf_spbox.value())

    def on_reverse(self):
        self.pm.reverse()

    def on_shift(self):
        self.pm.set_shift(self.shift_spbox.value())

    def update(self):
        [w.blockSignals(True) for w in self.widgets]
        self.update_name()
        self.scale_spbox.setValue(self.pm.get_scale())
        self.kerf_spbox.setValue(self.pm.get_kerf_width())
        self.shift_spbox.setValue(self.pm.get_shift())
        [w.blockSignals(False) for w in self.widgets]

    def update_name(self):
        self.name.setStyleSheet("color: rgb" + str(self.pm.color))
        if self.pm.loaded:
            self.name.setText(self.pm.name)
        else:
            self.name.setText('No path loaded')

class SyncViewWidget(QtGui.QWidget):
    def __init__(self, pm, color):
        super().__init__()
        self.pm = pm

        self.curve = pg.PlotCurveItem([], [], pen=pg.mkPen(color=color, width=2))
        self.add_pen = pg.mkPen(color=(80, 200, 0), width=2)
        self.remove_pen = pg.mkPen(color=(255, 0, 0), width=1)
        self.add_brush = pg.mkBrush(None)
        self.remove_brush = pg.mkBrush(color=(255,0,0))

        self.cursor_item = pg.ScatterPlotItem(pen=self.add_pen, brush=pg.mkBrush(None), size=10)
        self.sync_points_item = pg.ScatterPlotItem(pen=pg.mkPen(color=(0, 100, 200), width=2), brush=pg.mkBrush(None), size=10, symbol='o')

        self.plot = pg.PlotWidget()
        self.plot.addItem(self.curve)
        self.plot.addItem(self.sync_points_item)
        self.plot.addItem(self.cursor_item)
        self.plot.setAspectLocked()
        self.plot.showGrid(True, True, 0.4)
        grid_levels = [(1000, 0),(100, 0),(10, 0), (5, 0)]
        self.plot.getAxis("bottom").setTickSpacing(levels=grid_levels)
        self.plot.getAxis("left").setTickSpacing(levels=grid_levels)
        self.plot.setRange(xRange=(0, 100),yRange=(0, 50), padding=0, disableAutoRange=False)

        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.plot)
        self.setLayout(layout)
        self.pm.reset.connect(self.drawCurve)

        self.moveproxy = pg.SignalProxy(self.plot.scene().sigMouseMoved, rateLimit=60, slot=self.mouseMoved)
        self.clickproxy = pg.SignalProxy(self.plot.scene().sigMouseClicked, rateLimit=60, slot=self.mouseClicked)
        self.sync_points = np.array([[],[]])
        self.snap_pixels_len = 20

    def mouseMoved(self, evt):
        vb = self.plot.plotItem.getViewBox()
        snap_dist = vb.viewPixelSize()[0] * self.snap_pixels_len
        mpos = vb.mapSceneToView(evt[0])
        mpos = (mpos.x(), mpos.y())
        closest_point, closest_sync_point = self.pm.close_to(mpos)
        if closest_sync_point is not None and closest_sync_point['dist'] < snap_dist:
            self.cursor = closest_sync_point['pos'].reshape(2,1)
            self.cursor_type = 2
        elif closest_point is not None and closest_point['dist'] < snap_dist:
            self.cursor = closest_point['pos'].reshape(2,1)
            self.cursor_type = 1
        else:
            self.cursor_type = 0

        self.drawPoint()

    def mouseClicked(self, evt):
        if evt[0].button() != 1:
            return
        vb = self.plot.plotItem.getViewBox()
        snap_dist = vb.viewPixelSize()[0] * self.snap_pixels_len
        mpos = vb.mapSceneToView(evt[0].scenePos())
        mpos = (mpos.x(), mpos.y())
        closest_point, sync_points = self.pm.close_to(mpos)

        if sync_points is not None and sync_points['dist'] < snap_dist:
            self.pm.remove_sync_point(sync_points['deg'])
            self.cursor = closest_point['pos'].reshape(2,1)
            self.cursor_type = 1
        elif closest_point is not None and closest_point['dist'] < snap_dist:
            self.pm.add_sync_point(closest_point['deg'])
            self.cursor_type = 2
        else:
            self.cursor_type = 0
        self.drawPoint()

    def drawCurve(self):
        path = self.pm.gen.generate()
        if path.size > 0:
            self.curve.setData(path[0], path[1])
            # clear other items for autoRange on path only
            self.cursor_item.setData([], [])
            self.sync_points_item.setData([],[])
            self.plot.plotItem.getViewBox().autoRange()
        else:
            self.curve.setData([], [])
        self.cursor_type = 0
        self.drawPoint()

    def drawPoint(self):
        self.sync_points = self.pm.gen.sync_points_pos()
        self.sync_points_item.setData(self.sync_points[0], self.sync_points[1])
        if self.cursor_type == 0:
            self.cursor_item.setData([], [])
        elif self.cursor_type == 1:
            self.cursor_item.setPen(self.add_pen)
            self.cursor_item.setData(self.cursor[0], self.cursor[1], symbol = 'o', size=10, brush=self.add_brush)
        elif self.cursor_type == 2:
            self.cursor_item.setPen(self.remove_pen)
            self.cursor_item.setData(self.cursor[0], self.cursor[1], symbol='x', size=15, brush=self.remove_brush)
