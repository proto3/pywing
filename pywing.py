#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from PyQt5 import QtCore, QtGui, Qt
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np
import sys, time, queue, math
import serial.tools.list_ports

from airfoil import AirfoilModel

airfoil_data_folder = QtCore.QDir.homePath() + "/.airfoils"

class InterpolationModel(QtCore.QObject):
    data_changed = QtCore.pyqtSignal()
    bloc_changed = QtCore.pyqtSignal()
    bloc_changed_internal = QtCore.pyqtSignal()

    def __init__(self, machine_model, abs_path, rel_path):
        super().__init__()
        self._machine_model = machine_model

        self._path_generator_r = self._path_generator_abs = abs_path
        self._path_generator_l = self._path_generator_rel = rel_path

        self.abs_on_right = True

        self.lead_in_out = 10.0
        self._path_generator_abs.set_lead_size(self.lead_in_out)
        self._path_generator_rel.set_lead_size(self.lead_in_out)

        self.rel_rot = 0.0
        self.rel_tx  = 0.0
        self.rel_ty  = 0.0
        self.abs_rot = 0.0
        self.abs_tx  = 100.0 + self.lead_in_out
        self.abs_ty  = 0.0
        self._apply_transform()

        self._path_l = self._path_r = np.array([[],[],[]])
        self._synced_path_l = self._synced_path_r = np.array([[],[],[]])
        self._machine_path_l = self._machine_path_r = np.array([[],[],[]])

        self._bloc_width = 80.0
        self._bloc_offset = (self._machine_model.get_width()-self._bloc_width)/2

        self._path_generator_l.data_changed.connect(self._connect_paths)
        self._path_generator_r.data_changed.connect(self._connect_paths)
        self.bloc_changed.connect(self._connect_paths)

    def _connect_paths(self):
        if self.is_synced():
            degrees = list(set(self._path_generator_l.get_degree_list() + self._path_generator_r.get_degree_list()))
            degrees.sort()

            self._synced_path_l = np.insert(self._path_generator_l.get_interpolated_points(degrees), 1, self._bloc_offset + self._bloc_width, axis=0)
            self._synced_path_r = np.insert(self._path_generator_r.get_interpolated_points(degrees), 1, self._bloc_offset, axis=0)

            # gen machine path
            #TODO to be cleaned
            width = self._machine_model.get_width()
            left = self._synced_path_l
            right = self._synced_path_r
            self._machine_path_l = np.vstack(((left[0]-right[0])/(left[1]-right[1])*(width-left[1])+left[0], (left[2]-right[2])/(left[1]-right[1])*(width-left[1])+left[2]))
            self._machine_path_l = np.insert(self._machine_path_l, 1, width, axis=0)
            self._machine_path_r = np.vstack(((right[0]-left[0])/(right[1]-left[1])*(0.0-right[1])+right[0], (right[2]-left[2])/(right[1]-left[1])*(0.0-right[1])+right[2]))
            self._machine_path_r = np.insert(self._machine_path_r, 1, 0.0, axis=0)

        self._path_l = np.insert(self._path_generator_l.get_path(), 1, self._bloc_offset + self._bloc_width, axis=0)
        self._path_r = np.insert(self._path_generator_r.get_path(), 1, self._bloc_offset, axis=0)

        self.data_changed.emit()

    def generate_gcode(self, feedrate):
        gcode = list()
        if(self._path_generator_l.loaded and self._path_generator_r.loaded):
            prev_pos = (self._machine_path_r[0][0],
                        self._machine_path_r[2][0],
                        self._machine_path_l[0][0],
                        self._machine_path_l[2][0])
            prev_pos_s = (self._synced_path_r[0][0],
                          self._synced_path_r[2][0],
                          self._synced_path_l[0][0],
                          self._synced_path_l[2][0])
            gcode.append("G01 F%.3f X%.3f Y%.3f U%.3f V%.3f\n" % ((feedrate,) + prev_pos))


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
                gcode.append("G01 F%.3f X%.3f Y%.3f U%.3f V%.3f\n" % ((m_dist / s_dist * feedrate,) + new_pos))

        return gcode

    def is_synced(self):
        return self._path_generator_l.loaded and self._path_generator_r.loaded

    def get_paths(self):
        return (self._path_l, self._path_r)

    def get_synced_paths(self):
        return (self._synced_path_l, self._synced_path_r)

    def get_machine_paths(self):
        return (self._machine_path_l, self._machine_path_r)

    def get_synced_boundaries(self):
        bounds_r = self._path_generator_r.get_boundaries()
        bounds_l = self._path_generator_l.get_boundaries()
        return np.concatenate((np.minimum(bounds_r[:2], bounds_l[:2]), np.maximum(bounds_r[2:], bounds_l[2:])))

    def get_machine_boundaries(self):
        m_r = np.delete(self._machine_path_r, 1, 0)
        m_l = np.delete(self._machine_path_l, 1, 0)
        bounds_r = np.concatenate((np.amin(m_r, axis=1), np.amax(m_r, axis=1)))
        bounds_l = np.concatenate((np.amin(m_l, axis=1), np.amax(m_l, axis=1)))
        return np.concatenate((np.minimum(bounds_r[:2], bounds_l[:2]), np.maximum(bounds_r[2:], bounds_l[2:])))

    def set_lead_size(self, lead):
        self.lead_in_out = lead
        self._path_generator_abs.set_lead_size(self.lead_in_out)
        self._path_generator_rel.set_lead_size(self.lead_in_out)

    def set_bloc_width(self, width):
        self._bloc_width = width
        self.bloc_changed.emit()

    def set_bloc_offset(self, offset):
        self._bloc_offset = offset
        self.bloc_changed.emit()

    def _apply_transform(self):
        self._path_generator_abs.rotate(self.abs_rot)
        self._path_generator_rel.rotate(self.abs_rot + self.rel_rot)

        self._path_generator_abs.translate_x(self.abs_tx)
        self._path_generator_abs.translate_y(self.abs_ty)
        rrad = self.abs_rot / 180 * math.pi
        c = math.cos(rrad)
        s = math.sin(rrad)
        x = self.rel_tx * c - self.rel_ty * s
        y = self.rel_tx * s + self.rel_ty * c
        self._path_generator_rel.translate_x(self.abs_tx + x)
        self._path_generator_rel.translate_y(self.abs_ty + y)

    def rotate_abs(self, r):
        self.abs_rot = r
        self._apply_transform()

    def rotate_rel(self, r):
        self.rel_rot = r
        self._apply_transform()

    def translate_x_abs(self, tx):
        self.abs_tx = tx
        self._apply_transform()

    def translate_x_rel(self, tx):
        self.rel_tx = tx
        self._apply_transform()

    def translate_y_abs(self, ty):
        self.abs_ty = ty
        self._apply_transform()

    def translate_y_rel(self, ty):
        self.rel_ty = ty
        self._apply_transform()

    def is_abs_on_right(self):
        return self.abs_on_right

    def switch_ref_side(self):
        self.abs_on_right = not self.abs_on_right
        if self.abs_on_right:
            self._path_generator_l = self._path_generator_rel
            self._path_generator_r = self._path_generator_abs
        else:
            self._path_generator_l = self._path_generator_abs
            self._path_generator_r = self._path_generator_rel

        self._bloc_offset = self._machine_model.get_width() - self._bloc_width - self._bloc_offset
        self._path_generator_l.data_changed.emit()
        self.bloc_changed_internal.emit()

class MachineModel(QtCore.QObject):
    state_changed = QtCore.pyqtSignal()
    properties_changed = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self._wire_position = (0.0, 0.0, 0.0, 0.0)
        self._dimensions = (1000.0, 647.0, 400.0)

    def set_wire_position(self, position):
        self._wire_position = position
        self.state_changed.emit()

    def get_wire_position(self):
        return self._wire_position

    def set_no_wire_position(self):
        self._wire_position = None
        self.state_changed.emit()

    def set_dimensions(self, length, width, height):
        self._dimensions = (length, width, height)
        self.properties_changed.emit()

    def get_dimensions(self):
        return self._dimensions

    def get_width(self):
        return self._dimensions[1]

class AirfoilWidget(QtGui.QWidget):
    def __init__(self, airfoil, color):
        super().__init__()
        self._airfoil = airfoil

        self.name = QtGui.QLabel(text="No airfoil loaded")
        self.name.setAlignment(Qt.Qt.AlignCenter)
        self.name.setStyleSheet("color: rgb" + str(color))

        self.load_btn = QtGui.QPushButton("Load")
        self.load_btn.clicked.connect(self.on_load)

        self.scale_spbox = QtGui.QDoubleSpinBox()
        self.scale_spbox.setRange(1, 10000)
        self.scale_spbox.setValue(self._airfoil.s)
        self.scale_spbox.setPrefix("S : ")
        self.scale_spbox.setSuffix("mm")
        self.scale_spbox.valueChanged.connect(self.on_scale)

        self.dilate_spbox = QtGui.QDoubleSpinBox()
        self.dilate_spbox.setRange(0, 100)
        self.dilate_spbox.setValue(self._airfoil.k)
        self.dilate_spbox.setSingleStep(0.1)
        self.dilate_spbox.setPrefix("D : ")
        self.dilate_spbox.setSuffix("mm")
        self.dilate_spbox.valueChanged.connect(self.on_dilate)

        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.name)
        layout.addWidget(self.load_btn)
        layout.addWidget(self.scale_spbox)
        layout.addWidget(self.dilate_spbox)
        layout.addStretch()
        self.setLayout(layout)

    def on_load(self):
        filename, _ = QtGui.QFileDialog.getOpenFileName(self.load_btn.parent(), "Open File", airfoil_data_folder, "All Files (*)")
        if filename:
            self._airfoil.load(filename)
            self.name.setText(self._airfoil.name)

    def on_scale(self):
        self._airfoil.scale(self.scale_spbox.value())

    def on_dilate(self):
        self._airfoil.set_kerf_width(self.dilate_spbox.value())

class GraphicViewWidget(gl.GLViewWidget):

    def __init__(self, interpolation_model, machine, abs_color, rel_color):
        super().__init__()
        self._itpl_model = interpolation_model
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

        self._abs_color = abs_color
        self._rel_color = rel_color
        abs_right = self._itpl_model.is_abs_on_right()
        color_r = abs_color_fp if abs_right else rel_color_fp
        color_l = rel_color_fp if abs_right else abs_color_fp
        self.path_item_r = gl.GLLinePlotItem(color=color_r, width=3.0, antialias=True, mode='line_strip')
        self.path_item_l = gl.GLLinePlotItem(color=color_l, width=3.0, antialias=True, mode='line_strip')

        self.connection_lines_item = gl.GLLinePlotItem(color=(0.3, 0.0, 0.7, 0.5), antialias=True, mode='lines')
        self.wire_lines_item = gl.GLLinePlotItem(color=(0.7, 0.7, 0.7, 0.1), antialias=True, mode='lines')
        self.bloc = gl.GLLinePlotItem(color=(0.0, 0.4, 0.1, 1.0), width=2.0, antialias=True, mode='lines')
        self.under_bloc = gl.GLLinePlotItem(color=(0.0, 0.5, 0.6, 1.0), width=2.0, antialias=True, mode='lines')
        self.machine_item = gl.GLLinePlotItem(color=(1.0, 0.0, 0.0, 1.0), antialias=True, mode='lines')

        self._machine_path_item_r = gl.GLLinePlotItem(color=(0.8,0.0,0.2,1.0), width=2.0, antialias=True, mode='line_strip')
        self._machine_path_item_l = gl.GLLinePlotItem(color=(0.8,0.0,0.2,1.0), width=2.0, antialias=True, mode='line_strip')

        self._axis = gl.GLAxisItem(QtGui.QVector3D(50,50,50), glOptions='opaque')#size=None, antialias=True, glOptions='translucent')

        self.addItem(self._axis)
        self.addItem(right_plane)
        self.addItem(left_plane)
        self.addItem(bottom_plane)
        self.addItem(self.connection_lines_item)
        self.addItem(self.wire_lines_item)
        # self.addItem(self.bloc)
        # self.addItem(self.under_bloc)
        self.addItem(self.machine_item)
        self.addItem(self._machine_path_item_r)
        self.addItem(self._machine_path_item_l)

        self._itpl_model.data_changed.connect(self.draw_paths)
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
        paths = self._itpl_model.get_paths()
        abs_right = self._itpl_model.is_abs_on_right()
        color_r = abs_color_fp if abs_right else rel_color_fp
        color_l = rel_color_fp if abs_right else abs_color_fp
        self.path_item_l.setData(pos=paths[0].transpose(), color=color_l)
        self.path_item_r.setData(pos=paths[1].transpose(), color=color_r)

        if(not self._itpl_model.is_synced()):
            return

        machine_paths = self._itpl_model.get_machine_paths()
        self._machine_path_item_l.setData(pos = machine_paths[0].transpose())
        self._machine_path_item_r.setData(pos = machine_paths[1].transpose())

        # wire_lines = np.concatenate((machine_paths[0].transpose(), machine_paths[1].transpose()), axis=1).reshape(len(machine_paths[0][0])*2,3)
        # self.wire_lines_item.setData(pos=wire_lines)

        synced_paths = self._itpl_model.get_synced_paths()
        connection_lines = np.concatenate((synced_paths[0].transpose(), synced_paths[1].transpose()), axis=1).reshape(len(synced_paths[0][0])*2,3)
        self.connection_lines_item.setData(pos=connection_lines)

        limits = self._itpl_model.get_synced_boundaries()

        length, width, height = machine.get_dimensions()

        left_bloc = self._itpl_model._bloc_offset+self._itpl_model._bloc_width
        right_bloc = self._itpl_model._bloc_offset
        vertices = np.array([
            [limits[0], left_bloc, limits[1]],
            [limits[0], right_bloc, limits[1]],
            [limits[0], left_bloc, limits[3]],
            [limits[0], right_bloc, limits[3]],
            [limits[2], left_bloc, limits[3]],
            [limits[2], right_bloc, limits[3]],
            [limits[2], left_bloc, limits[1]],
            [limits[2], right_bloc, limits[1]]])

        indices = [0,1,2,3,4,5,6,7,0,2,2,4,4,6,6,0,1,3,3,5,5,7,7,1]
        bloc_lines = np.take(vertices, axis=0, indices=indices)
        # print("length from", limits[0],"to", limits[2])
        # print("height from", limits[1],"to", limits[3])
        self.bloc.setData(pos=bloc_lines)

        if limits[1] > 0.0:
            vertices = np.array([
                [limits[0], left_bloc,         0],
                [limits[0], right_bloc,         0],
                [limits[0], left_bloc, limits[1]],
                [limits[0], right_bloc, limits[1]],
                [limits[2], left_bloc, limits[1]],
                [limits[2], right_bloc, limits[1]],
                [limits[2], left_bloc,         0],
                [limits[2], right_bloc,         0]])
            indices = [0,1,6,7,0,2,4,6,6,0,1,3,5,7,7,1]
            bloc_lines = np.take(vertices, axis=0, indices=indices)
            self.under_bloc.setData(pos=bloc_lines)
        else:
            self.under_bloc.setData(pos=None)

class SerialThread(QtCore.QThread):
    connection_changed = QtCore.pyqtSignal()
    port_list_changed = QtCore.pyqtSignal()

    def __init__(self, machine):
        super().__init__()
        self._machine = machine
        self.port = ""
        self.port_list = []

        self.connected = False
        self.connecting = False
        self.running = False
        self.stop_request = False
        self.connect_request = False
        self.disconnect_request = False
        self.gcode = []
        self.last_status_request = time.time()

        self.on_board_buf = 128
        self.past_cmd_len = queue.Queue()

    def __del__(self):
        self.wait()

    def connect(self, port):
        if(not self.connected):
            self.port = port
            self.connect_request = True
        else:
            print("already connected")
            pass

    def disconnect(self):
        if(self.connected):
            if(not self.running):
                self.disconnect_request = True
            else:
                print("running")
                pass
        else:
            print("not connected")
            pass

    def play(self, gcode):
        if(self.connected):
            if(not self.running):
                self.gcode = list(gcode)
                self.running = True
            else:
                print("already running")
                pass
        else:
            print("not connected")
            pass

    def stop(self):
        if(self.connected):
            if(self.running):
                self.stop_request = True
            else:
                print("not running")
                pass
        else:
            print("not connected")
            pass

    def run(self):
        while(True):
            if(self.connected):
                if(self.disconnect_request):
                    self._reset()

                try:
                    if(self.stop_request):
                        self.serial.write(("!").encode("ascii"))
                        self.running = False
                        self.stop_request = False
                except serial.SerialException:
                    self._reset()
                    continue

                try:
                    if(self.running):
                        if(self.gcode):
                            if(len(self.gcode[0]) <= self.on_board_buf):
                                cmd = self.gcode.pop(0)
                                self.serial.write(cmd.encode("ascii"))
                                self.on_board_buf -= len(cmd)
                                self.past_cmd_len.put(len(cmd))
                        else:
                            self.running = False
                except serial.SerialException:
                    self._reset()
                    continue

                try:
                    now = time.time()
                    if(self.last_status_request + 0.2 < now):
                        self.serial.write(("?").encode("ascii"))
                        self.last_status_request = now
                except serial.SerialException:
                    self._reset()
                    continue

                try:
                    read_data = self.serial.readline().decode("ascii")
                    self._process_read_data(read_data)
                except serial.SerialException:
                    self._reset()
                    continue

            else:
                if(self.connect_request):
                    self._attempt_connection(self.port)
                    self.connect_request = False
                else:
                    self.port_list = [port.device for port in serial.tools.list_ports.comports()]
                    self.port_list.sort()
                    self.port_list_changed.emit()
                    time.sleep(0.2)

    def _reset(self):
        self.serial.close()
        self.running = False
        self.stop_request = False
        self.connect_request = False
        self.disconnect_request = False
        self.on_board_buf = 128
        self.past_cmd_len = queue.Queue()

        self.connected = False
        self._machine.set_no_wire_position()
        self.connection_changed.emit()

    def _process_read_data(self, data):
        if(data == 'ok\r\n'):
            self.on_board_buf += self.past_cmd_len.get()
        elif(data != ''):
            if(data[0] == "<"):
                self._parse_status(data)
            else:
                # handle grbl errors here
                pass

    def _parse_status(self, status):
        mpos_idx = status.find("MPos:")
        mpos_str = status[mpos_idx+5:].split("|")[0].split(",")
        mpos = [float(i) for i in mpos_str]
        if(mpos[0] == mpos[2] and mpos[1] == mpos[3]):
            mpos[3] += 0.001
        self._machine.set_wire_position(mpos)

    def _attempt_connection(self, port):
        self.connecting = True
        self.connection_changed.emit()
        try:
            self.serial = serial.Serial(port, 115200, timeout=2.0)
            crlf = self.serial.readline()
            prompt = self.serial.readline().decode("ascii")
            if(prompt[:4] == "Grbl"):
                self.serial.timeout = 0.1
                self.connected = True
            else:
                print("Prompt failed.")
        except:
            pass
        self.connecting = False
        self.connection_changed.emit()

class CuttingWidget(QtGui.QWidget):

    def __init__(self, interpolation_model, machine):
        super().__init__()

        self._itpl_model = interpolation_model
        self._machine = machine
        self.serial_thread = SerialThread(machine)
        self.serial_thread.connection_changed.connect(self.on_connection_change)
        self.serial_thread.port_list_changed.connect(self.on_port_list_change)
        self.serial_thread.start()

        self.play_btn = QtGui.QPushButton("play")
        self.play_btn.clicked.connect(self.on_play)
        self.stop_btn = QtGui.QPushButton("stop")
        self.stop_btn.clicked.connect(self.on_stop)
        self.connect_btn = QtGui.QPushButton("Connect")
        self.connect_btn.clicked.connect(self.on_connect)

        self.load_btn = QtGui.QPushButton("Load project")
        self.load_btn.clicked.connect(self.on_load)

        self.port_box = QtGui.QComboBox()
        self.port_box.setInsertPolicy(QtGui.QComboBox.InsertAlphabetically)
        self.port_box.setSizeAdjustPolicy(QtGui.QComboBox.AdjustToContents)

        self.feed_spbox = QtGui.QDoubleSpinBox()
        self.feed_spbox.setRange(1, 1000)
        self.feed_spbox.setValue(200)
        self.feed_spbox.setSingleStep(10)
        self.feed_spbox.setPrefix("Feedrate : ")
        self.feed_spbox.setSuffix("mm/s")

        self.serial_text_item = QtGui.QTextEdit()
        self.serial_data = ""

        self.lead_spbox = QtGui.QDoubleSpinBox()
        self.lead_spbox.setRange(1, 1000)
        self.lead_spbox.setValue(self._itpl_model.lead_in_out)
        self.lead_spbox.setPrefix("Lead in/out : ")
        self.lead_spbox.setSuffix("mm")
        self.lead_spbox.valueChanged.connect(self.on_lead_change)

        self.bloc_width_spbox = QtGui.QDoubleSpinBox()
        self.bloc_width_spbox.setRange(1, self._machine.get_width())
        self.bloc_width_spbox.setValue(self._itpl_model._bloc_width)
        self.bloc_width_spbox.setPrefix("Bloc width : ")
        self.bloc_width_spbox.setSuffix("mm")
        self.bloc_width_spbox.valueChanged.connect(self.on_bloc_width_change)

        self.bloc_offset_spbox = QtGui.QDoubleSpinBox()
        self.bloc_offset_spbox.setRange(0, self._machine.get_width() - self.bloc_width_spbox.value())
        self.bloc_offset_spbox.setValue(self._itpl_model._bloc_offset)
        self._itpl_model.bloc_changed_internal.connect(self.set_bloc_offset_display)
        self.bloc_offset_spbox.setPrefix("Bloc offset : ")
        self.bloc_offset_spbox.setSuffix("mm")
        self.bloc_offset_spbox.valueChanged.connect(self.on_bloc_offset_change)

        layout = QtGui.QGridLayout()
        layout.addWidget(self.feed_spbox, 0, 0)
        layout.addWidget(self.lead_spbox, 1, 0)
        layout.addWidget(self.bloc_offset_spbox, 2, 0)
        layout.addWidget(self.bloc_width_spbox, 3, 0)
        layout.addWidget(self.play_btn, 4, 0)
        layout.addWidget(self.stop_btn, 5, 0)
        layout.addWidget(self.serial_text_item, 0, 1, 6, 1)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 5)
        layout.addWidget(self.port_box, 0, 6)
        layout.addWidget(self.connect_btn, 1, 6)
        layout.addWidget(self.load_btn, 3, 6)

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

    def on_load(self):
        pass

    def on_play(self):
        gcode = self._itpl_model.generate_gcode(self.feed_spbox.value())

        self.serial_data = ""
        self.serial_data += ";Left  airfoil : " + self._itpl_model._path_generator_l.name
        self.serial_data += (" | R:%.2f S%.2f TX%.2f TY%.2f D%.2f\n" %
                (self._itpl_model._path_generator_l.r,
                self._itpl_model._path_generator_l.s,
                self._itpl_model._path_generator_l.t[0],
                self._itpl_model._path_generator_l.t[1],
                self._itpl_model._path_generator_l.k))
        self.serial_data += ";Right airfoil : " + self._itpl_model._path_generator_r.name
        self.serial_data += (" | R:%.2f S%.2f TX%.2f TY%.2f D%.2f\n" %
                (self._itpl_model._path_generator_r.r,
                self._itpl_model._path_generator_r.s,
                self._itpl_model._path_generator_r.t[0],
                self._itpl_model._path_generator_r.t[1],
                self._itpl_model._path_generator_r.k))

        for command in gcode:
            self.serial_data += command
        self.serial_text_item.setText(self.serial_data)

        self.serial_thread.play(gcode)

    def on_lead_change(self):
        self._itpl_model.set_lead_size(self.lead_spbox.value())

    def on_bloc_width_change(self):
        self._itpl_model.set_bloc_width(self.bloc_width_spbox.value())
        self.bloc_offset_spbox.setMaximum(self._machine.get_width() - self.bloc_width_spbox.value())

    def on_bloc_offset_change(self):
        self._itpl_model.set_bloc_offset(self.bloc_offset_spbox.value())

    def set_bloc_offset_display(self):
        self.bloc_offset_spbox.setValue(self._itpl_model._bloc_offset)

class RelativeSettingsWidget(QtGui.QWidget):

    def __init__(self, interpolation_model):
        super().__init__()

        self._itpl_model = interpolation_model

        self.name = QtGui.QLabel("Relative", alignment=Qt.Qt.AlignCenter)

        self.rot_spbox = QtGui.QDoubleSpinBox()
        self.rot_spbox.setRange(-90, 90)
        self.rot_spbox.setSingleStep(0.1)
        self.rot_spbox.setValue(self._itpl_model.rel_rot)
        self.rot_spbox.setPrefix("R : ")
        self.rot_spbox.setSuffix("°")
        self.rot_spbox.valueChanged.connect(self.on_rot)

        self.tx_spbox = QtGui.QDoubleSpinBox()
        self.tx_spbox.setRange(-10000, 10000)
        self.tx_spbox.setValue(self._itpl_model.rel_tx)
        self.tx_spbox.setPrefix("TX : ")
        self.tx_spbox.setSuffix("mm")
        self.tx_spbox.valueChanged.connect(self.on_tx)

        self.ty_spbox = QtGui.QDoubleSpinBox()
        self.ty_spbox.setRange(-10000, 10000)
        self.ty_spbox.setValue(self._itpl_model.rel_ty)
        self.ty_spbox.setPrefix("TY : ")
        self.ty_spbox.setSuffix("mm")
        self.ty_spbox.valueChanged.connect(self.on_ty)

        self.switch_btn = QtGui.QPushButton("Switch")
        self.switch_btn.clicked.connect(self.on_switch)

        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.name)
        layout.addWidget(self.rot_spbox)
        layout.addWidget(self.tx_spbox)
        layout.addWidget(self.ty_spbox)
        layout.addWidget(self.switch_btn)
        self.setLayout(layout)

    def on_rot(self):
        self._itpl_model.rotate_rel(self.rot_spbox.value())

    def on_tx(self):
        self._itpl_model.translate_x_rel(self.tx_spbox.value())

    def on_ty(self):
        self._itpl_model.translate_y_rel(self.ty_spbox.value())

    def on_switch(self):
        self._itpl_model.switch_ref_side()
        airfoil_widget_switch.switch()

class AbsoluteSettingsWidget(QtGui.QWidget):

    def __init__(self, interpolation_model):
        super().__init__()

        self._itpl_model = interpolation_model

        self.name = QtGui.QLabel("Absolute", alignment=Qt.Qt.AlignCenter)

        self.rot_spbox = QtGui.QDoubleSpinBox()
        self.rot_spbox.setRange(-90, 90)
        self.rot_spbox.setSingleStep(0.1)
        self.rot_spbox.setValue(self._itpl_model.abs_rot)
        self.rot_spbox.setPrefix("R : ")
        self.rot_spbox.setSuffix("°")
        self.rot_spbox.valueChanged.connect(self.on_rot)

        self.tx_spbox = QtGui.QDoubleSpinBox()
        self.tx_spbox.setRange(-10000, 10000)
        self.tx_spbox.setValue(self._itpl_model.abs_tx)
        self.tx_spbox.setPrefix("TX : ")
        self.tx_spbox.setSuffix("mm")
        self.tx_spbox.valueChanged.connect(self.on_tx)

        self.ty_spbox = QtGui.QDoubleSpinBox()
        self.ty_spbox.setRange(-10000, 10000)
        self.ty_spbox.setValue(self._itpl_model.abs_ty)
        self.ty_spbox.setPrefix("TY : ")
        self.ty_spbox.setSuffix("mm")
        self.ty_spbox.valueChanged.connect(self.on_ty)

        self.auto_align_btn = QtGui.QPushButton("Auto align")
        self.auto_align_btn.clicked.connect(self.on_auto_align)

        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.name)
        layout.addWidget(self.rot_spbox)
        layout.addWidget(self.tx_spbox)
        layout.addWidget(self.ty_spbox)
        layout.addWidget(self.auto_align_btn)
        self.setLayout(layout)

    def on_rot(self):
        self._itpl_model.rotate_abs(self.rot_spbox.value())

    def on_tx(self):
        self._itpl_model.translate_x_abs(self.tx_spbox.value())

    def on_ty(self):
        self._itpl_model.translate_y_abs(self.ty_spbox.value())

    def on_auto_align(self):
        if(self._itpl_model.is_synced()):
            margin = 5
            bndr = self._itpl_model.get_machine_boundaries()
            absset_widget.tx_spbox.setValue(self.tx_spbox.value()-bndr[0] + margin)
            absset_widget.ty_spbox.setValue(self.ty_spbox.value()-bndr[1] + margin)

class AirfoilWidgetSwitch():
    def __init__(self, itpl_model, layout, airfoil_widget_abs, airfoil_widget_rel):
        self._itpl_model = itpl_model
        self._layout = layout
        self._airfoil_widget_abs = airfoil_widget_abs
        self._airfoil_widget_rel = airfoil_widget_rel

    def switch(self):
        abs_on_right = self._itpl_model.is_abs_on_right()
        self._layout.removeWidget(self._airfoil_widget_abs)
        self._layout.removeWidget(self._airfoil_widget_rel)
        self._layout.addWidget(self._airfoil_widget_abs,0,3 if abs_on_right else 0)
        self._layout.addWidget(self._airfoil_widget_rel,0,0 if abs_on_right else 3)

if __name__ == '__main__':
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOption('antialias', True)
    app = QtGui.QApplication([])

    abs_color = (233, 79, 55)
    rel_color = (46, 134, 171)
    abs_color_fp = tuple(i/255 for i in abs_color) + (1.0,)
    rel_color_fp = tuple(i/255 for i in rel_color) + (1.0,)

    machine = MachineModel()
    airfoil_abs = AirfoilModel()
    airfoil_rel = AirfoilModel()
    airfoil_widget_abs = AirfoilWidget(airfoil_abs, abs_color)
    airfoil_widget_rel = AirfoilWidget(airfoil_rel, rel_color)
    itpl_model = InterpolationModel(machine, abs_path = airfoil_abs, rel_path = airfoil_rel)

    graphic_view_widget = GraphicViewWidget(itpl_model, machine, abs_color_fp, rel_color_fp)
    cutting_widget = CuttingWidget(itpl_model, machine)
    relset_widget = RelativeSettingsWidget(itpl_model)
    absset_widget = AbsoluteSettingsWidget(itpl_model)

    top_widget = QtGui.QWidget()
    grid_layout = QtGui.QGridLayout()
    grid_layout.addWidget(airfoil_widget_rel,0,0)
    grid_layout.addWidget(relset_widget,1,0)
    grid_layout.addWidget(graphic_view_widget,0,1,2,2)
    grid_layout.setColumnStretch(1, 1)
    grid_layout.addWidget(airfoil_widget_abs,0,3)
    grid_layout.addWidget(absset_widget,1,3)
    top_widget.setLayout(grid_layout)
    airfoil_widget_switch = AirfoilWidgetSwitch(itpl_model, grid_layout, airfoil_widget_abs, airfoil_widget_rel)

    main_widget = QtGui.QWidget()
    layout = QtGui.QVBoxLayout()
    layout.addWidget(top_widget)
    layout.addWidget(cutting_widget)
    main_widget.setLayout(layout)
    main_widget.show()

    sys.exit(app.exec_())
