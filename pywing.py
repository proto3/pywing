#!/usr/bin/python3
# -*- coding: utf-8 -*-
from PyQt5 import QtCore, QtGui, Qt
import pyqtgraph as pg
import sys

from airfoil import Airfoil

airfoil_data_folder = QtCore.QDir.homePath() + "/.airfoils"
airfoil_left = Airfoil()
airfoil_right = Airfoil()

class AirfoilManager:
    def __init__(self, airfoil, color, name):
        self.airfoil = airfoil
        self.curve_item = pg.PlotCurveItem([], [], pen=pg.mkPen(color=color, width=2))

        self.load_btn = QtGui.QPushButton("Load " + name)
        self.load_btn.clicked.connect(self.on_load)

        self.rot_slider = QtGui.QSlider()
        self.rot_slider.setMinimum(-90)
        self.rot_slider.setMaximum(90)
        self.rot_slider.setValue(0)
        self.rot_slider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.rot_slider.setTickInterval(10)
        self.rot_slider.valueChanged.connect(self.on_rot)

    def on_load(self):
        filename, _ = QtGui.QFileDialog.getOpenFileName(self.load_btn.parent(), "Open File", airfoil_data_folder, "All Files (*)")
        if filename:
            self.airfoil.load(filename)
            self.curve_item.setData(self.airfoil.x, self.airfoil.y)

    def on_rot(self):
        self.airfoil.rotate(self.rot_slider.value())
        self.curve_item.setData(self.airfoil.x, self.airfoil.y)

class MainWidget(QtGui.QWidget):
    def __init__(self, filename = None):
        super().__init__()

        self.airfoil_left_view = AirfoilManager(airfoil_left,(46, 134, 171), "left")
        self.airfoil_right_view = AirfoilManager(airfoil_right,(233, 79, 55), "right")

        plot = pg.PlotWidget()
        plot.addItem(self.airfoil_left_view.curve_item)
        plot.addItem(self.airfoil_right_view.curve_item)

        grid_alpha = 50
        grid_levels = [(10, 0), (5, 0), (1, 0)]
        x = plot.getAxis("bottom")
        y = plot.getAxis("left")
        x.setGrid(grid_alpha)
        y.setGrid(grid_alpha)
        x.setTickSpacing(levels=grid_levels)
        y.setTickSpacing(levels=grid_levels)

        plot.setRange(xRange=(0, 100), padding=0, disableAutoRange=False)
        plot.setAspectLocked()

        layout = QtGui.QGridLayout()
        layout.addWidget(self.airfoil_left_view.load_btn, 0, 0)
        layout.addWidget(self.airfoil_left_view.rot_slider, 1, 0)
        layout.addWidget(plot, 0, 1, 3, 1)
        layout.addWidget(self.airfoil_right_view.load_btn, 0, 2)
        layout.addWidget(self.airfoil_right_view.rot_slider, 1, 2)
        self.setLayout(layout)

if __name__ == '__main__':
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOption('antialias', True)

    app = QtGui.QApplication([])
    main_widget = MainWidget()
    main_widget.show()
    sys.exit(app.exec_())
