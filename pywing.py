#!/usr/bin/python3
# -*- coding: utf-8 -*-
from PyQt5 import QtCore, QtGui, Qt
import pyqtgraph as pg
import sys, os
import numpy as np

from airfoil import Airfoil

airfoil_data_folder = QtCore.QDir.homePath() + "/.airfoils"

airfoil_left = Airfoil()
airfoil_right = Airfoil()

fusion = pg.QtGui.QGraphicsPathItem()

def airfoil_refresh_callback():
    if(airfoil_left.loaded and airfoil_right.loaded):
        l = airfoil_left.get_point_chord_list() + airfoil_right.get_point_chord_list()
        l = list(set(l))
        l.sort()

        airfoil_right.get_points(l)
        airfoil_left.get_points(l)

        x = np.dstack((airfoil_right.spe_data[0], airfoil_left.spe_data[0]))
        y = np.dstack((airfoil_right.spe_data[1], airfoil_left.spe_data[1]))
        connect = np.dstack((np.ones(len(l)), np.zeros(len(l))))

        path = pg.arrayToQPath(x.flatten(), y.flatten(), connect.flatten())
        fusion.setPath(path)


class AirfoilItemManager:
    def __init__(self, airfoil, color):
        self.airfoil = airfoil
        self.curve = pg.PlotCurveItem([], [], pen=pg.mkPen(color=color, width=2))
        self.dot_curve = pg.ScatterPlotItem([], [], pen=pg.mkPen(color=color, width=2))

        self.load_btn = QtGui.QPushButton("Load")
        self.load_btn.clicked.connect(self.on_load)

        self.rot_spbox = QtGui.QDoubleSpinBox()
        self.rot_spbox.setRange(-90, 90)
        self.rot_spbox.setValue(airfoil.r)
        self.rot_spbox.setPrefix("R : ")
        self.rot_spbox.setSuffix("°")
        self.rot_spbox.valueChanged.connect(self.on_rot)

        self.scale_spbox = QtGui.QDoubleSpinBox()
        self.scale_spbox.setRange(0, 10000)
        self.scale_spbox.setValue(airfoil.s)
        self.scale_spbox.setPrefix("S : ")
        self.scale_spbox.setSuffix("mm")
        self.scale_spbox.valueChanged.connect(self.on_scale)

        self.tx_spbox = QtGui.QDoubleSpinBox()
        self.tx_spbox.setRange(-10000, 10000)
        self.tx_spbox.setValue(airfoil.t[0])
        self.tx_spbox.setPrefix("TX : ")
        self.tx_spbox.setSuffix("mm")
        self.tx_spbox.valueChanged.connect(self.on_tx)

        self.ty_spbox = QtGui.QDoubleSpinBox()
        self.ty_spbox.setRange(-10000, 10000)
        self.ty_spbox.setValue(airfoil.t[1])
        self.ty_spbox.setPrefix("TY : ")
        self.ty_spbox.setSuffix("mm")
        self.ty_spbox.valueChanged.connect(self.on_ty)

        self.dilate_spbox = QtGui.QDoubleSpinBox()
        self.dilate_spbox.setRange(0, 100)
        self.dilate_spbox.setValue(airfoil.d)
        self.dilate_spbox.setSingleStep(0.1)
        self.dilate_spbox.setPrefix("D : ")
        self.dilate_spbox.setSuffix("mm")
        self.dilate_spbox.valueChanged.connect(self.on_dilate)

        self.name = QtGui.QLabel(text="No airfoil loaded")
        self.name.setAlignment(Qt.Qt.AlignCenter)
        self.name.setMaximumSize(1000, 20)
        color_str = "(" + str(color[0]) + "," + str(color[1]) + "," + str(color[2]) + ")"
        self.name.setStyleSheet("color: rgb" + color_str)

    def __refresh_curve(self):
        self.curve.setData(self.airfoil.trans_data[0], self.airfoil.trans_data[1])
        self.dot_curve.setData(self.airfoil.spe_data[0], self.airfoil.spe_data[1])
        airfoil_refresh_callback()

    def on_load(self):
        filename, _ = QtGui.QFileDialog.getOpenFileName(self.load_btn.parent(), "Open File", airfoil_data_folder, "All Files (*)")
        if filename:
            self.airfoil.load(filename)
            self.__refresh_curve()
            self.name.setText(os.path.splitext(os.path.basename(filename))[0])

    def on_rot(self):
        self.airfoil.rotate(self.rot_spbox.value())
        self.__refresh_curve()

    def on_scale(self):
        self.airfoil.scale(self.scale_spbox.value())
        self.__refresh_curve()

    def on_tx(self):
        self.airfoil.translate_x(self.tx_spbox.value())
        self.__refresh_curve()

    def on_ty(self):
        self.airfoil.translate_y(self.ty_spbox.value())
        self.__refresh_curve()

    def on_dilate(self):
        self.airfoil.dilate(self.dilate_spbox.value())
        self.__refresh_curve()

class MainWidget(QtGui.QWidget):
    def __init__(self, filename = None):
        super().__init__()

        self.airfoil_left_view = AirfoilItemManager(airfoil_left,(46, 134, 171))
        self.airfoil_right_view = AirfoilItemManager(airfoil_right,(233, 79, 55))

        plot = pg.PlotWidget()
        fusion.setPen(pg.mkPen(0.7))
        plot.addItem(fusion)
        plot.addItem(self.airfoil_left_view.curve)
        plot.addItem(self.airfoil_left_view.dot_curve)
        plot.addItem(self.airfoil_right_view.curve)
        plot.addItem(self.airfoil_right_view.dot_curve)


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
        layout.addWidget(self.airfoil_left_view.name, 0, 0)
        layout.addWidget(self.airfoil_left_view.load_btn, 1, 0)
        layout.addWidget(self.airfoil_left_view.rot_spbox, 2, 0)
        layout.addWidget(self.airfoil_left_view.scale_spbox, 3, 0)
        layout.addWidget(self.airfoil_left_view.tx_spbox, 4, 0)
        layout.addWidget(self.airfoil_left_view.ty_spbox, 5, 0)
        layout.addWidget(self.airfoil_left_view.dilate_spbox, 6, 0)
        layout.addWidget(plot, 0, 1, 8, 1)
        layout.addWidget(self.airfoil_right_view.name, 0, 2)
        layout.addWidget(self.airfoil_right_view.load_btn, 1, 2)
        layout.addWidget(self.airfoil_right_view.rot_spbox, 2, 2)
        layout.addWidget(self.airfoil_right_view.scale_spbox, 3, 2)
        layout.addWidget(self.airfoil_right_view.tx_spbox, 4, 2)
        layout.addWidget(self.airfoil_right_view.ty_spbox, 5, 2)
        layout.addWidget(self.airfoil_right_view.dilate_spbox, 6, 2)
        self.setLayout(layout)

if __name__ == '__main__':
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOption('antialias', True)

    app = QtGui.QApplication([])
    main_widget = MainWidget()
    main_widget.show()
    sys.exit(app.exec_())
