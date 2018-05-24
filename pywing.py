#!/usr/bin/python3
# -*- coding: utf-8 -*-
from PyQt5 import QtCore, QtGui, Qt
import pyqtgraph as pg
import sys
import numpy as np
import time
import serial.tools.list_ports
import queue
import glob
import enum

from airfoil import Airfoil

airfoil_data_folder = QtCore.QDir.homePath() + "/.airfoils"

class ConnectedAirfoilsModel(QtCore.QObject):
    data_changed = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self.af_left = Airfoil()
        self.af_right = Airfoil()
        self.it_point_list_right = []
        self.it_point_list_left = []

        self.af_left.data_changed.connect(self.connect_airfoils)
        self.af_right.data_changed.connect(self.connect_airfoils)

    def connect_airfoils(self):
        if(self.af_left.loaded and self.af_right.loaded):
            degrees = list(set(self.af_left.get_degree_list() + self.af_right.get_degree_list()))
            degrees.sort()

            self.it_point_list_right = self.af_right.get_interpolated_point_list(degrees)
            self.it_point_list_left = self.af_left.get_interpolated_point_list(degrees)

            self.data_changed.emit()

    def generate_gcode(self, feedrate):
        gcode = list()
        if(self.af_left.loaded and self.af_right.loaded):
            gcode.append("F%.2f\n" % feedrate)
            gcode.append("G01 X%.3f Y%.3f U%.3f V%.3f\n" %
                    (self.af_right.start[0],
                    self.af_right.start[1],
                    self.af_left.start[0],
                    self.af_left.start[1]))

            for i in range(len(self.it_point_list_right[0])):
                gcode.append("G01 X%.3f Y%.3f U%.3f V%.3f\n" %
                    (self.it_point_list_right[0][i],
                    self.it_point_list_right[1][i],
                    self.it_point_list_left[0][i],
                    self.it_point_list_left[1][i]))

            gcode.append("G01 X%.3f Y%.3f U%.3f V%.3f\n" %
                    (self.af_right.end[0],
                    self.af_right.end[1],
                    self.af_left.end[0],
                    self.af_left.end[1]))

        return gcode

class MachineModel(QtCore.QObject):
    data_changed = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self._position = [0.0, 0.0, 0.0, 0.0]

    def set_position(self, position):
        self._position = position
        self.data_changed.emit()

    def get_position(self):
        return self._position

    def set_no_position(self):
        self._position = None
        self.data_changed.emit()

class AirfoilItemManager:
    def __init__(self, airfoil, color):
        self.airfoil = airfoil
        self.airfoil.data_changed.connect(self.draw)

        self.curve = pg.PlotCurveItem([], [], pen=pg.mkPen(color=color, width=2))
        self.lead_in = pg.PlotCurveItem([], [], pen=pg.mkPen(color=color, width=3, style=QtCore.Qt.DotLine))
        self.lead_out = pg.PlotCurveItem([], [], pen=pg.mkPen(color=color, width=3, style=QtCore.Qt.DotLine))

        self.load_btn = QtGui.QPushButton("Load")
        self.load_btn.clicked.connect(self.on_load)

        self.rot_spbox = QtGui.QDoubleSpinBox()
        self.rot_spbox.setRange(-90, 90)
        self.rot_spbox.setValue(self.airfoil.r)
        self.rot_spbox.setPrefix("R : ")
        self.rot_spbox.setSuffix("°")
        self.rot_spbox.valueChanged.connect(self.on_rot)

        self.scale_spbox = QtGui.QDoubleSpinBox()
        self.scale_spbox.setRange(1, 10000)
        self.scale_spbox.setValue(self.airfoil.s)
        self.scale_spbox.setPrefix("S : ")
        self.scale_spbox.setSuffix("mm")
        self.scale_spbox.valueChanged.connect(self.on_scale)

        self.tx_spbox = QtGui.QDoubleSpinBox()
        self.tx_spbox.setRange(-10000, 10000)
        self.tx_spbox.setValue(self.airfoil.t[0])
        self.tx_spbox.setPrefix("TX : ")
        self.tx_spbox.setSuffix("mm")
        self.tx_spbox.valueChanged.connect(self.on_tx)

        self.ty_spbox = QtGui.QDoubleSpinBox()
        self.ty_spbox.setRange(-10000, 10000)
        self.ty_spbox.setValue(self.airfoil.t[1])
        self.ty_spbox.setPrefix("TY : ")
        self.ty_spbox.setSuffix("mm")
        self.ty_spbox.valueChanged.connect(self.on_ty)

        self.dilate_spbox = QtGui.QDoubleSpinBox()
        self.dilate_spbox.setRange(0, 100)
        self.dilate_spbox.setValue(self.airfoil.d)
        self.dilate_spbox.setSingleStep(0.1)
        self.dilate_spbox.setPrefix("D : ")
        self.dilate_spbox.setSuffix("mm")
        self.dilate_spbox.valueChanged.connect(self.on_dilate)

        self.name = QtGui.QLabel(text="No airfoil loaded")
        self.name.setAlignment(Qt.Qt.AlignCenter)
        self.name.setMaximumSize(1000, 20)
        self.name.setStyleSheet("color: rgb" + str(color))

    def draw(self):
        self.curve.setData(self.airfoil.trans_data[0], self.airfoil.trans_data[1]) #TODO ensure this is atomic
        self.lead_in.setData([self.airfoil.start[0], self.airfoil.trans_data[0][0]], [self.airfoil.start[1], self.airfoil.trans_data[1][0]])
        self.lead_out.setData([self.airfoil.end[0], self.airfoil.trans_data[0][-1]], [self.airfoil.end[1], self.airfoil.trans_data[1][-1]])

    def on_load(self):
        filename, _ = QtGui.QFileDialog.getOpenFileName(self.load_btn.parent(), "Open File", airfoil_data_folder, "All Files (*)")
        if filename:
            self.airfoil.load(filename)
            self.name.setText(self.airfoil.name)

    def on_rot(self):
        self.airfoil.rotate(self.rot_spbox.value())

    def on_scale(self):
        self.airfoil.scale(self.scale_spbox.value())

    def on_tx(self):
        self.airfoil.translate_x(self.tx_spbox.value())

    def on_ty(self):
        self.airfoil.translate_y(self.ty_spbox.value())

    def on_dilate(self):
        self.airfoil.dilate(self.dilate_spbox.value())

class SideViewWidget(QtGui.QWidget):

    def __init__(self, connected_airfoils, machine):
        super().__init__()
        plot = pg.PlotWidget()

        # TODO test not to save them and send all data through signal (perf ?)
        self._connected_airfoils = connected_airfoils
        self._machine = machine

        self.aim_left = AirfoilItemManager(self._connected_airfoils.af_left,(46, 134, 171))
        self.aim_right = AirfoilItemManager(self._connected_airfoils.af_right,(233, 79, 55))
        self.connection_lines_item = pg.QtGui.QGraphicsPathItem()
        self.connection_lines_item.setPen(pg.mkPen(0.7))
        self.machine_item = pg.PlotCurveItem([], [], pen=pg.mkPen(color=(84, 209, 35), width=4))

        self._connected_airfoils.data_changed.connect(self.draw_connected)
        self._machine.data_changed.connect(self.draw_machine)

        plot.addItem(self.connection_lines_item)
        plot.addItem(self.aim_left.curve)
        plot.addItem(self.aim_right.curve)
        plot.addItem(self.aim_left.lead_in)
        plot.addItem(self.aim_right.lead_in)
        plot.addItem(self.aim_left.lead_out)
        plot.addItem(self.aim_right.lead_out)
        plot.addItem(self.machine_item)

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
        layout.addWidget(self.aim_left.name, 0, 0)
        layout.addWidget(self.aim_left.load_btn, 1, 0)
        layout.addWidget(self.aim_left.rot_spbox, 2, 0)
        layout.addWidget(self.aim_left.scale_spbox, 3, 0)
        layout.addWidget(self.aim_left.tx_spbox, 4, 0)
        layout.addWidget(self.aim_left.ty_spbox, 5, 0)
        layout.addWidget(self.aim_left.dilate_spbox, 6, 0)
        layout.addWidget(plot, 0, 1, 8, 1)
        layout.addWidget(self.aim_right.name, 0, 2)
        layout.addWidget(self.aim_right.load_btn, 1, 2)
        layout.addWidget(self.aim_right.rot_spbox, 2, 2)
        layout.addWidget(self.aim_right.scale_spbox, 3, 2)
        layout.addWidget(self.aim_right.tx_spbox, 4, 2)
        layout.addWidget(self.aim_right.ty_spbox, 5, 2)
        layout.addWidget(self.aim_right.dilate_spbox, 6, 2)
        self.setLayout(layout)

    def draw_machine(self):
        mpos = self._machine.get_position()
        if(mpos is not None):
            self.machine_item.setData([mpos[0],mpos[2]], [mpos[1],mpos[3]])
        else:
            self.machine_item.setData([])

    def draw_connected(self):
        right_points = self._connected_airfoils.it_point_list_right
        left_points = self._connected_airfoils.it_point_list_left
        nb_points = len(left_points[0])

        x = np.dstack((right_points[0], left_points[0]))
        y = np.dstack((right_points[1], left_points[1]))
        connected = np.dstack((np.ones(nb_points), np.zeros(nb_points)))

        path = pg.arrayToQPath(x.flatten(), y.flatten(), connected.flatten())
        self.connection_lines_item.setPath(path)

class SerialThread(QtCore.QThread):
    connection_changed = QtCore.pyqtSignal()

    def __init__(self, machine):
        super().__init__()
        self._machine = machine
        self.port = ""

        self.connected = False
        self.running = False
        self.stop_request = False
        self.connect_request = False
        self.disconnect_request = False
        self.gcode = []
        self.last_status_request = time.time()

        self.on_board_buf = 128
        self.past_cmd_len = queue.Queue()

        # self.connection_changed.connect(self.on_connection_change)

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
                    ports = glob.glob('/dev/tty[A-Za-z]*')
                    result = []
                    for port in ports:
                        try:
                            s = serial.Serial(port)
                            s.close()
                            result.append(port)
                        except (OSError, serial.SerialException):
                            pass
                    print(result)
                    # TODO update spbox with ports
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
        self.connection_changed.emit()

    def _process_read_data(self, data):
        if(data == 'ok\r\n'):
            self.on_board_buf += self.past_cmd_len.get()
        elif(data != ''):
            if(data[0] == "<"):
                self._parse_status(data)
            else:
                pass #TODO handle grbl errors

    def _parse_status(self, status):
        mpos_idx = status.find("MPos:")
        mpos_str = status[mpos_idx+5:].split("|")[0].split(",")
        mpos = [float(i) for i in mpos_str]
        if(mpos[0] == mpos[2] and mpos[1] == mpos[3]):
            mpos[3] += 0.001
        self._machine.set_position(mpos)

    def _attempt_connection(self, port):
        try:
            self.serial = serial.Serial(port, 115200, timeout=2.0)
            crlf = self.serial.readline()
            prompt = self.serial.readline().decode("ascii")
            if(prompt[:4] == "Grbl"):
                self.serial.timeout = 0.1
                self.connected = True
                self.connection_changed.emit()
            else:
                print("Prompt failed.")
        except:
            pass

    #TODO connecting state

    # TODO
    # def on_connection_change(self):
    #     if(self.connection != ConnectEnum.Connected):
    #         self._machine.set_no_position()

class ControlWidget(QtGui.QWidget):

    def __init__(self, connected_airfoils, machine):
        super().__init__()

        self._connected_airfoils = connected_airfoils
        self.serial_thread = SerialThread(machine)
        self.serial_thread.connection_changed.connect(self.on_connection_change)
        self.serial_thread.start()

        layout = QtGui.QGridLayout()
        play_btn = QtGui.QPushButton("play")
        play_btn.clicked.connect(self.on_play)
        stop_btn = QtGui.QPushButton("stop")
        stop_btn.clicked.connect(self.on_stop)
        self.connect_btn = QtGui.QPushButton("Connect")
        self.connect_btn.clicked.connect(self.on_connect)

        self.port_box = QtGui.QComboBox()
        self.port_box.resize(300,120)
        port_list = [port.device for port in serial.tools.list_ports.comports()]
        port_list.sort()
        for port_name in port_list:
            self.port_box.addItem(port_name)

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
        self.lead_spbox.setValue(self._connected_airfoils.af_left.lead_in_out)
        self.lead_spbox.setPrefix("Lead in/out : ")
        self.lead_spbox.setSuffix("mm")
        self.lead_spbox.valueChanged.connect(self.on_lead_change)

        layout.addWidget(self.feed_spbox, 0, 0)
        layout.addWidget(self.lead_spbox, 1, 0)
        layout.addWidget(play_btn, 1, 0)
        layout.addWidget(stop_btn, 3, 0)
        layout.addWidget(self.serial_text_item, 0, 1, 5, 1)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 5)
        layout.addWidget(self.port_box, 0, 6)
        layout.addWidget(self.connect_btn, 1, 6)
        self.setLayout(layout)

    def serial_ports(self):
        ports = glob.glob('/dev/tty[A-Za-z]*')
        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result

    def on_connection_change(self):
        if(self.serial_thread.connected):
            text = "Disconnect"
            self.connect_btn.setFlat(False)
        # elif(self.serial_thread.connection == ConnectEnum.Connecting):
        #     text = "Connecting..."
        #     self.connect_btn.setFlat(True)
        else:
            text = "Connect"
            self.connect_btn.setFlat(False)
        self.connect_btn.setText(text)

    def on_stop(self):
        self.serial_thread.stop()

    def on_connect(self):
        if(self.serial_thread.connected):
            self.serial_thread.disconnect()
        else:
            self.serial_thread.connect(self.port_box.currentText())

    def on_play(self):
        gcode = self._connected_airfoils.generate_gcode(self.feed_spbox.value())

        self.serial_data = ""
        self.serial_data += ";Left  airfoil : " + self._connected_airfoils.af_left.name
        self.serial_data += (" | R:%.2f S%.2f TX%.2f TY%.2f D%.2f\n" %
                (self._connected_airfoils.af_left.r,
                self._connected_airfoils.af_left.s,
                self._connected_airfoils.af_left.t[0],
                self._connected_airfoils.af_left.t[1],
                self._connected_airfoils.af_left.d))
        self.serial_data += ";Right airfoil : " + self._connected_airfoils.af_right.name
        self.serial_data += (" | R:%.2f S%.2f TX%.2f TY%.2f D%.2f\n" %
                (self._connected_airfoils.af_right.r,
                self._connected_airfoils.af_right.s,
                self._connected_airfoils.af_right.t[0],
                self._connected_airfoils.af_right.t[1],
                self._connected_airfoils.af_right.d))

        for command in gcode:
            self.serial_data += command
        self.serial_text_item.setText(self.serial_data)

        self.serial_thread.play(gcode)

    def on_lead_change(self):
        self._connected_airfoils.af_left.set_lead(self.lead_spbox.value())
        self._connected_airfoils.af_right.set_lead(self.lead_spbox.value())

class MainWidget(QtGui.QSplitter):
    def __init__(self, w1, w2):
        super().__init__(QtCore.Qt.Vertical)
        self.addWidget(w1)
        self.addWidget(w2)

if __name__ == '__main__':
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOption('antialias', True)
    app = QtGui.QApplication([])

    connect_airfoils = ConnectedAirfoilsModel()
    machine = MachineModel()

    side_view_widget = SideViewWidget(connect_airfoils, machine)
    control_widget = ControlWidget(connect_airfoils, machine)

    main_widget = MainWidget(side_view_widget, control_widget)
    main_widget.show()

    sys.exit(app.exec_())
