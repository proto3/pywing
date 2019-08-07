#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from PyQt5 import QtCore, QtGui, Qt

class PositionModel(QtCore.QObject):
    update = QtCore.pyqtSignal()
    reset  = QtCore.pyqtSignal()

    def __init__(self, name, r=0.0, t=[0.0, 0.0]):
        super().__init__()
        self.name = name
        self.r = r
        self.t = t

    def export_tuple(self):
        return (self.name, self.r, self.t)

    def import_tuple(self, tuple):
        self.name, self.r, self.t = tuple
        self.reset.emit()

    def rotate(self, r):
        self.r = r
        self.update.emit()

    def translate_x(self, tx):
        self.t[0] = tx
        self.update.emit()

    def translate_y(self, ty):
        self.t[1] = ty
        self.update.emit()

class PositionWidget(QtGui.QWidget):
    def __init__(self, position_model):
        super().__init__()
        self.pos = position_model

        self.name = QtGui.QLabel(self.pos.name, alignment=Qt.Qt.AlignCenter)

        self.rot_spbox = QtGui.QDoubleSpinBox()
        self.rot_spbox.setRange(-90, 90)
        self.rot_spbox.setSingleStep(0.1)
        self.rot_spbox.setValue(self.pos.r)
        self.rot_spbox.setPrefix("R : ")
        self.rot_spbox.setSuffix("°")
        self.rot_spbox.valueChanged.connect(self.on_rot)

        self.tx_spbox = QtGui.QDoubleSpinBox()
        self.tx_spbox.setRange(-10000, 10000)
        self.tx_spbox.setValue(self.pos.t[0])
        self.tx_spbox.setPrefix("TX : ")
        self.tx_spbox.setSuffix("mm")
        self.tx_spbox.valueChanged.connect(self.on_tx)

        self.ty_spbox = QtGui.QDoubleSpinBox()
        self.ty_spbox.setRange(-10000, 10000)
        self.ty_spbox.setValue(self.pos.t[1])
        self.ty_spbox.setPrefix("TY : ")
        self.ty_spbox.setSuffix("mm")
        self.ty_spbox.valueChanged.connect(self.on_ty)

        self.widgets = (self.name, self.rot_spbox, self.tx_spbox, self.ty_spbox)

        layout = QtGui.QVBoxLayout()
        [layout.addWidget(w) for w in self.widgets]
        self.setLayout(layout)

        self.pos.reset.connect(self.reset)

    def on_rot(self):
        self.pos.rotate(self.rot_spbox.value())

    def on_tx(self):
        self.pos.translate_x(self.tx_spbox.value())

    def on_ty(self):
        self.pos.translate_y(self.ty_spbox.value())

    def reset(self):
        [w.blockSignals(True) for w in self.widgets]
        self.name.setText(self.pos.name)
        self.rot_spbox.setValue(self.pos.r)
        self.tx_spbox.setValue(self.pos.t[0])
        self.ty_spbox.setValue(self.pos.t[1])
        [w.blockSignals(False) for w in self.widgets]
