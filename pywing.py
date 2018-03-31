#!/usr/bin/python3
# -*- coding: utf-8 -*-
from PyQt5 import QtCore, QtGui
import pyqtgraph as pg
import sys

from airfoil import Airfoil

airfoil_data_folder = QtCore.QDir.homePath() + "/.airfoils"

def load_airfoil(filename):
    af = Airfoil(filename)
    af.normalize()
    af *= 100
    fusion = af.merge(af)
    fusion[0] = 100 - fusion[0]
    return (fusion[0], fusion[1])


class MyWidget(QtGui.QWidget):
    def __init__(self, filename = None):
        super().__init__()

        left_color = (46, 134, 171)
        right_color = (233, 79, 55)
        self.airfoil_left = pg.PlotCurveItem([], [], pen=pg.mkPen(color=left_color, width=2))
        self.airfoil_right = pg.PlotCurveItem([], [], pen=pg.mkPen(color=right_color, width=2))

        load_left_btn = QtGui.QPushButton('Load left')
        load_right_btn = QtGui.QPushButton('Load right')
        load_left_btn.clicked.connect(self.on_load_left)
        load_right_btn.clicked.connect(self.on_load_right)

        plot = pg.PlotWidget()
        plot.addItem(self.airfoil_left)
        plot.addItem(self.airfoil_right)

        grid_alpha = 50
        grid_levels = [(10, 0), (5, 0), (1, 0)]
        x = plot.getAxis("bottom")
        y = plot.getAxis("left")
        x.setGrid(grid_alpha)
        y.setGrid(grid_alpha)
        x.setTickSpacing(levels=grid_levels)
        y.setTickSpacing(levels=grid_levels)

        plot.setRange(xRange=(0,100), padding=0, disableAutoRange=False)
        plot.setAspectLocked()

        layout = QtGui.QGridLayout()
        layout.addWidget(load_left_btn, 0, 0)
        layout.addWidget(load_right_btn, 1, 0)
        layout.addWidget(plot, 0, 1, 3, 1)
        self.setLayout(layout)

    def open_airfoil(self, airfoil):
        filename, _ = QtGui.QFileDialog.getOpenFileName(self, "Open File", airfoil_data_folder, "All Files (*)")
        if filename:
            a, b = load_airfoil(filename)
            airfoil.setData(a, b)

    def on_load_left(self):
        self.open_airfoil(self.airfoil_left)

    def on_load_right(self):
        self.open_airfoil(self.airfoil_right)

if __name__ == '__main__':
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOption('antialias', True)

    app = QtGui.QApplication([])
    main_widget = MyWidget()
    main_widget.show()
    sys.exit(app.exec_())
