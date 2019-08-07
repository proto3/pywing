#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from PyQt5 import QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np
import pickle
import sys

from airfoil import *
from machine import *
from foamblock import *
from position import *
from cutparameters import *

class CutProcessor(QtCore.QObject):
    update = QtCore.pyqtSignal()

    def __init__(self, machine_model, l_path, r_path, abs_pos_model, rel_pos_model, foam_block_model, cut_param_model):
        super().__init__()
        self._machine_model = machine_model

        self.l_path_gen = self.rel_path_gen = l_path
        self.r_path_gen = self.abs_path_gen = r_path

        self.abs_on_right = True

        self.foam_block = foam_block_model
        self.cut_param = cut_param_model

        self.abs_pos = abs_pos_model
        self.rel_pos = rel_pos_model
        self.abs_pos.import_tuple((self.abs_pos.name,
                                   self.abs_pos.r,
                                   [100.0 + self.cut_param.lead, 0.0]))
        self._apply_transform()

        self._path_l = self._path_r = np.array([[],[],[]])
        self._synced_path_l = self._synced_path_r = np.array([[],[],[]])
        self._machine_path_l = self._machine_path_r = np.array([[],[],[]])

        self.l_path_gen.update.connect(self._connect_paths)
        self.r_path_gen.update.connect(self._connect_paths)

        self.abs_pos.update.connect(self._apply_transform)
        self.rel_pos.update.connect(self._apply_transform)
        self.cut_param.update.connect(self._apply_transform)
        self.foam_block.update.connect(self._connect_paths)

    def _connect_paths(self):
        if self.is_synced():
            degrees = list(set(self.l_path_gen.get_degree_list() + self.r_path_gen.get_degree_list()))
            degrees.sort()

            self._synced_path_l = np.insert(self.l_path_gen.get_interpolated_points(degrees), 1, self.foam_block.offset + self.foam_block.width, axis=0)
            self._synced_path_r = np.insert(self.r_path_gen.get_interpolated_points(degrees), 1, self.foam_block.offset, axis=0)

            # gen machine path
            #TODO to be cleaned
            width = self._machine_model.get_width()
            left = self._synced_path_l
            right = self._synced_path_r
            self._machine_path_l = np.vstack(((left[0]-right[0])/(left[1]-right[1])*(width-left[1])+left[0], (left[2]-right[2])/(left[1]-right[1])*(width-left[1])+left[2]))
            self._machine_path_l = np.insert(self._machine_path_l, 1, width, axis=0)
            self._machine_path_r = np.vstack(((right[0]-left[0])/(right[1]-left[1])*(0.0-right[1])+right[0], (right[2]-left[2])/(right[1]-left[1])*(0.0-right[1])+right[2]))
            self._machine_path_r = np.insert(self._machine_path_r, 1, 0.0, axis=0)

        self._path_l = np.insert(self.l_path_gen.get_path(), 1, self.foam_block.offset + self.foam_block.width, axis=0)
        self._path_r = np.insert(self.r_path_gen.get_path(), 1, self.foam_block.offset, axis=0)

        self.update.emit()

    def generate_gcode(self):
        gcode = list()
        if(self.l_path_gen.loaded and self.r_path_gen.loaded):
            prev_pos = (self._machine_path_r[0][0],
                        self._machine_path_r[2][0],
                        self._machine_path_l[0][0],
                        self._machine_path_l[2][0])
            prev_pos_s = (self._synced_path_r[0][0],
                          self._synced_path_r[2][0],
                          self._synced_path_l[0][0],
                          self._synced_path_l[2][0])
            gcode.append("G01 F%.3f X%.3f Y%.3f U%.3f V%.3f\n" % ((self.cut_param.feedrate,) + prev_pos))


            for i in range(1, len(self._synced_path_r[0])):
                new_pos = (self._machine_path_r[0][i],
                           self._machine_path_r[2][i],
                           self._machine_path_l[0][i],
                           self._machine_path_l[2][i])
                new_pos_s = (self._synced_path_r[0][i],
                             self._synced_path_r[2][i],
                             self._synced_path_l[0][i],
                             self._synced_path_l[2][i])
                machine_diff = np.array(new_pos) - np.array(prev_pos)
                synced_diff = np.array(new_pos_s) - np.array(prev_pos_s)
                m_square = np.square(machine_diff)
                s_square = np.square(synced_diff)
                m_dist = max(math.sqrt(m_square[0]+m_square[1]), math.sqrt(m_square[2]+m_square[3]))
                s_dist = max(math.sqrt(s_square[0]+s_square[1]), math.sqrt(s_square[2]+s_square[3]))

                prev_pos = new_pos
                prev_pos_s = new_pos_s
                gcode.append("G01 F%.3f X%.3f Y%.3f U%.3f V%.3f\n" % ((m_dist / s_dist * self.cut_param.feedrate,) + new_pos))

        program = str()
        program += ";Left  airfoil: " + self.l_path_gen.name
        program += (" | S: %.2f R: %.2f TX: %.2f TY: %.2f K: %.2f\n" %
                (self.l_path_gen.s,
                self.l_path_gen.r,
                self.l_path_gen.t[0],
                self.l_path_gen.t[1],
                self.l_path_gen.k))
        program += ";Right airfoil : " + self.r_path_gen.name
        program += (" | S: %.2f R: %.2f TX: %.2f TY: %.2f K: %.2f\n" %
                (self.r_path_gen.s,
                self.r_path_gen.r,
                self.r_path_gen.t[0],
                self.r_path_gen.t[1],
                self.r_path_gen.k))

        for command in gcode:
            program += command

        return program

    def is_synced(self):
        return self.l_path_gen.loaded and self.r_path_gen.loaded

    def get_path_colors(self):
        return (self.l_path_gen.color, self.r_path_gen.color)

    def get_paths(self):
        return (self._path_l, self._path_r)

    def get_synced_paths(self):
        return (self._synced_path_l, self._synced_path_r)

    def get_machine_paths(self):
        return (self._machine_path_l, self._machine_path_r)

    def get_synced_boundaries(self):
        bounds_r = self.r_path_gen.get_boundaries()
        bounds_l = self.l_path_gen.get_boundaries()
        return np.concatenate((np.minimum(bounds_r[:2], bounds_l[:2]), np.maximum(bounds_r[2:], bounds_l[2:])))

    def get_machine_boundaries(self):
        m_r = np.delete(self._machine_path_r, 1, 0)
        m_l = np.delete(self._machine_path_l, 1, 0)
        bounds_r = np.concatenate((np.amin(m_r, axis=1), np.amax(m_r, axis=1)))
        bounds_l = np.concatenate((np.amin(m_l, axis=1), np.amax(m_l, axis=1)))
        return np.concatenate((np.minimum(bounds_r[:2], bounds_l[:2]), np.maximum(bounds_r[2:], bounds_l[2:])))

    def _apply_transform(self):
        self.abs_path_gen.set_lead_size(self.cut_param.lead)
        self.rel_path_gen.set_lead_size(self.cut_param.lead)

        self.abs_path_gen.rotate(self.abs_pos.r)
        self.rel_path_gen.rotate(self.abs_pos.r + self.rel_pos.r)

        self.abs_path_gen.translate_x(self.abs_pos.t[0])
        self.abs_path_gen.translate_y(self.abs_pos.t[1])
        rrad = self.abs_pos.r / 180 * math.pi
        c = math.cos(rrad)
        s = math.sin(rrad)
        x = self.rel_pos.t[0] * c - self.rel_pos.t[1] * s
        y = self.rel_pos.t[0] * s + self.rel_pos.t[1] * c
        self.rel_path_gen.translate_x(self.abs_pos.t[0] + x)
        self.rel_path_gen.translate_y(self.abs_pos.t[1] + y)

    def is_abs_on_right(self):
        return self.abs_on_right

    def reverse(self):
        # exchange content of left and right paths
        tmp = self.l_path_gen.export_tuple()
        self.l_path_gen.import_tuple(self.r_path_gen.export_tuple())
        self.r_path_gen.import_tuple(tmp)

        # switch absolute side between left and right
        self.abs_on_right = not self.abs_on_right
        if self.abs_on_right:
            self.rel_path_gen = self.l_path_gen
            self.abs_path_gen = self.r_path_gen
        else:
            self.abs_path_gen = self.l_path_gen
            self.rel_path_gen = self.r_path_gen

        # apply relative and absolute position to paths
        self._apply_transform()

        # reverse block offset
        self.foam_block.reverse()

    def align(self):
        if(self.is_synced()):
            margin = 5
            bndr = self.get_machine_boundaries()
            self.abs_pos.import_tuple(
                (self.abs_pos.name,
                self.abs_pos.r,
                [self.abs_pos.t[0]-bndr[0] + margin,
                 self.abs_pos.t[1]-bndr[1] + margin]))
            self._apply_transform()

    def save(self, filename):
        fp = open(filename,'wb+')

        pickle.dump(self.abs_on_right, fp)
        pickle.dump(self.cut_param.export_tuple(), fp)
        pickle.dump(self.foam_block.export_tuple(), fp)
        pickle.dump(self.abs_pos.export_tuple(), fp)
        pickle.dump(self.rel_pos.export_tuple(), fp)
        pickle.dump(self.l_path_gen.export_tuple(), fp)
        pickle.dump(self.r_path_gen.export_tuple(), fp)

        fp.close()

    def load(self, filename):
        fp = open(filename, 'rb')

        self.abs_on_right = pickle.load(fp)
        self.cut_param.import_tuple(pickle.load(fp))
        self.foam_block.import_tuple(pickle.load(fp))
        self.abs_pos.import_tuple(pickle.load(fp))
        self.rel_pos.import_tuple(pickle.load(fp))
        self.l_path_gen.import_tuple(pickle.load(fp))
        self.r_path_gen.import_tuple(pickle.load(fp))

        fp.close()

        if self.abs_on_right:
            self.rel_path_gen = self.l_path_gen
            self.abs_path_gen = self.r_path_gen
        else:
            self.abs_path_gen = self.l_path_gen
            self.rel_path_gen = self.r_path_gen
        self._apply_transform()

class GraphicViewWidget(gl.GLViewWidget):

    def __init__(self, cut_processor, machine):
        super().__init__()
        self._cut_proc = cut_processor
        self._machine = machine

        length, width, height = self._machine.get_dimensions()
        # self.setBackgroundColor((210, 234, 239))
        self.setCameraPosition(distance=width+length, azimuth=225)
        self.pan(length/2, width/2, 0.0)

        ## create three grids, add each to the view
        right_plane  = gl.GLGridItem(QtGui.QVector3D(length, height, 0))
        left_plane   = gl.GLGridItem(QtGui.QVector3D(length, height, 0))
        bottom_plane = gl.GLGridItem(QtGui.QVector3D(length,  width, 0))
        right_plane.rotate(90, 1, 0, 0)
        left_plane.rotate(90, 1, 0, 0)
        right_plane.translate( length/2,  width, height/2)
        left_plane.translate(  length/2,      0, height/2)
        bottom_plane.translate(length/2,width/2,        0)
        right_plane.setSpacing(50, 50, 50)
        left_plane.setSpacing(50, 50, 50)
        bottom_plane.setSpacing(50, 50, 50)

        color_l, color_r = self._cut_proc.get_path_colors()
        l_color_fp = tuple(i/255 for i in color_l) + (1.0,)
        r_color_fp = tuple(i/255 for i in color_r) + (1.0,)
        self.path_item_l = gl.GLLinePlotItem(color=l_color_fp, width=3.0, antialias=True, mode='line_strip')
        self.path_item_r = gl.GLLinePlotItem(color=r_color_fp, width=3.0, antialias=True, mode='line_strip')

        self.connection_lines_item = gl.GLLinePlotItem(color=(0.3, 0.0, 0.7, 0.5), antialias=True, mode='lines')
        self.machine_item = gl.GLLinePlotItem(color=(1.0, 0.0, 0.0, 1.0), antialias=True, mode='lines')

        self._machine_path_item_r = gl.GLLinePlotItem(color=(0.8,0.0,0.2,1.0), width=2.0, antialias=True, mode='line_strip')
        self._machine_path_item_l = gl.GLLinePlotItem(color=(0.8,0.0,0.2,1.0), width=2.0, antialias=True, mode='line_strip')

        self._axis = gl.GLAxisItem(QtGui.QVector3D(50,50,50), glOptions='opaque')#size=None, antialias=True, glOptions='translucent')

        self.addItem(self._axis)
        self.addItem(right_plane)
        self.addItem(left_plane)
        self.addItem(bottom_plane)
        self.addItem(self.connection_lines_item)
        self.addItem(self.machine_item)
        self.addItem(self._machine_path_item_r)
        self.addItem(self._machine_path_item_l)

        self._cut_proc.update.connect(self.draw_paths)
        self._machine.properties_changed.connect(self.draw_paths)
        self._machine.state_changed.connect(self.draw_machine_state)

        self.addItem(self.path_item_l)
        self.addItem(self.path_item_r)

    def draw_machine_state(self):
        mpos = self._machine.get_wire_position()
        if(mpos is not None):
            self.machine_item.setData(pos=np.array(((mpos[0], 0.0, mpos[1]), (mpos[2], self._machine.get_width(), mpos[3]))))
        else:
            self.machine_item.setData(pos=np.array())

    def draw_paths(self):
        paths = self._cut_proc.get_paths()
        color_l, color_r = self._cut_proc.get_path_colors()
        l_color_fp = tuple(i/255 for i in color_l) + (1.0,)
        r_color_fp = tuple(i/255 for i in color_r) + (1.0,)
        self.path_item_l.setData(pos=paths[0].transpose(), color=l_color_fp)
        self.path_item_r.setData(pos=paths[1].transpose(), color=r_color_fp)

        if(not self._cut_proc.is_synced()):
            return

        machine_paths = self._cut_proc.get_machine_paths()
        self._machine_path_item_l.setData(pos = machine_paths[0].transpose())
        self._machine_path_item_r.setData(pos = machine_paths[1].transpose())

        synced_paths = self._cut_proc.get_synced_paths()
        connection_lines = np.concatenate((synced_paths[0].transpose(), synced_paths[1].transpose()), axis=1).reshape(len(synced_paths[0][0])*2,3)
        self.connection_lines_item.setData(pos=connection_lines)

class CuttingProcessorWidget(QtGui.QWidget):

    def __init__(self, cut_processor, machine):
        super().__init__()

        self._cut_proc = cut_processor
        self._machine = machine
        self.serial_thread = SerialThread(machine)
        self.serial_thread.connection_changed.connect(self.on_connection_change)
        self.serial_thread.port_list_changed.connect(self.on_port_list_change)
        self.serial_thread.start()

        self.reverse_btn = QtGui.QPushButton("Reverse")
        self.reverse_btn.clicked.connect(self.on_reverse)
        self.align_btn = QtGui.QPushButton("Auto align")
        self.align_btn.clicked.connect(self.on_align)
        self.save_btn = QtGui.QPushButton("Save project")
        self.save_btn.clicked.connect(self.on_save)
        self.load_btn = QtGui.QPushButton("Load project")
        self.load_btn.clicked.connect(self.on_load)
        self.connect_btn = QtGui.QPushButton("Connect")
        self.connect_btn.clicked.connect(self.on_connect)
        self.play_btn = QtGui.QPushButton("play")
        self.play_btn.clicked.connect(self.on_play)
        self.stop_btn = QtGui.QPushButton("stop")
        self.stop_btn.clicked.connect(self.on_stop)

        self.port_box = QtGui.QComboBox()
        self.port_box.setInsertPolicy(QtGui.QComboBox.InsertAlphabetically)
        self.port_box.setSizeAdjustPolicy(QtGui.QComboBox.AdjustToContents)

        self.serial_text_item = QtGui.QTextEdit()
        self.serial_data = ""

        layout = QtGui.QGridLayout()
        layout.addWidget(self.reverse_btn, 0, 0)
        layout.addWidget(self.align_btn, 1, 0)
        layout.addWidget(self.save_btn, 2, 0)
        layout.addWidget(self.load_btn, 3, 0)
        layout.addWidget(self.serial_text_item, 0, 1, 4, 1)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 5)
        layout.addWidget(self.port_box, 0, 6)
        layout.addWidget(self.connect_btn, 1, 6)
        layout.addWidget(self.play_btn, 2, 6)
        layout.addWidget(self.stop_btn, 3, 6)

        self.setLayout(layout)

    def on_connection_change(self):
        if(self.serial_thread.connecting):
            text = "Connecting..."
            self.connect_btn.setFlat(True)
        elif(self.serial_thread.connected):
            text = "Disconnect"
            self.connect_btn.setFlat(False)
        else:
            text = "Connect"
            self.connect_btn.setFlat(False)

        self.connect_btn.setText(text)

    def on_port_list_change(self):
        new_items = self.serial_thread.port_list
        prev_items = []
        for idx in range(self.port_box.count()):
            prev_items.append(self.port_box.itemText(idx))

        items_to_remove = list(set(prev_items) - set(new_items))
        items_to_insert = list(set(new_items) - set(prev_items))
        items_to_remove.sort()
        items_to_insert.sort()

        for item in items_to_remove:
            self.port_box.removeItem(self.port_box.findText(item))

        self.port_box.insertItems(0, items_to_insert)

    def on_stop(self):
        self.serial_thread.stop()

    def on_connect(self):
        if(self.serial_thread.connected):
            self.serial_thread.disconnect()
        else:
            self.serial_thread.connect(self.port_box.currentText())

    def on_save(self):
        filename, _ = QtGui.QFileDialog.getSaveFileName(self.save_btn.parent(), "Save project", QtCore.QDir.homePath() +"/example.pw", ".pw Files (*.pw) ;; All Files (*)")
        if filename:
            self._cut_proc.save(filename)

    def on_load(self):
        filename, _ = QtGui.QFileDialog.getOpenFileName(self.load_btn.parent(), "Open project", QtCore.QDir.homePath(), ".pw Files (*.pw) ;; All Files (*)")
        if filename:
            self._cut_proc.load(filename)

    def on_play(self):
        program = self._cut_proc.generate_gcode()
        self.serial_text_item.setText(program)
        self.serial_thread.play(program)

    def on_reverse(self):
        self._cut_proc.reverse()

    def on_align(self):
        self._cut_proc.align()

if __name__ == '__main__':
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOption('antialias', True)
    app = QtGui.QApplication([])

    abs_color = (233, 79, 55)
    rel_color = (46, 134, 171)

    machine = MachineModel()
    airfoil_l = AirfoilGenerator(rel_color)
    airfoil_r = AirfoilGenerator(abs_color)
    airfoil_widget_l = AirfoilWidget(airfoil_l)
    airfoil_widget_r = AirfoilWidget(airfoil_r)

    abs_pos_model = PositionModel('Absolute')
    abs_pos_widget = PositionWidget(abs_pos_model)
    rel_pos_model = PositionModel('Relative')
    rel_pos_widget = PositionWidget(rel_pos_model)
    foam_block_model = FoamBlockModel(machine)
    foam_block_widget = FoamBlockWidget(foam_block_model)
    cut_param_model = CutParametersModel()
    cut_param_widget = CutParametersWidget(cut_param_model)

    cut_proc = CutProcessor(machine, airfoil_l, airfoil_r, abs_pos_model, rel_pos_model, foam_block_model, cut_param_model)

    graphic_view_widget = GraphicViewWidget(cut_proc, machine)
    cutting_proc_widget = CuttingProcessorWidget(cut_proc, machine)

    top_widget = QtGui.QWidget()
    grid_layout = QtGui.QGridLayout()
    grid_layout.addWidget(airfoil_widget_l,0,0)
    grid_layout.addWidget(rel_pos_widget,1,0)
    grid_layout.addWidget(cut_param_widget,2,0)
    grid_layout.addWidget(graphic_view_widget,0,1,3,2)
    grid_layout.setColumnStretch(1, 1)
    grid_layout.addWidget(airfoil_widget_r,0,3)
    grid_layout.addWidget(abs_pos_widget,1,3)
    grid_layout.addWidget(foam_block_widget,2,3)
    top_widget.setLayout(grid_layout)

    main_widget = QtGui.QWidget()
    layout = QtGui.QVBoxLayout()
    layout.addWidget(top_widget)
    layout.addWidget(cutting_proc_widget)
    main_widget.setLayout(layout)
    main_widget.show()

    sys.exit(app.exec_())
