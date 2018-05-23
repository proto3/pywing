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
            for i in range(len(self.it_point_list_right[0])):
                gcode.append("G01 X%.3f Y%.3f U%.3f V%.3f\n" %
                    (self.it_point_list_right[0][i],
                    self.it_point_list_right[1][i],
                    self.it_point_list_left[0][i],
                    self.it_point_list_left[1][i]))
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

class AirfoilItemManager:
    def __init__(self, airfoil, color):
        self.airfoil = airfoil
        self.airfoil.data_changed.connect(self.draw)

        self.curve = pg.PlotCurveItem([], [], pen=pg.mkPen(color=color, width=2))

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
        self.machine_item.setData([mpos[0],mpos[2]], [mpos[1],mpos[3]])

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

    def __init__(self, machine):
        super().__init__()
        self.command_list = queue.Queue()
        self.port = ""
        self.play_request = False
        self._machine = machine
        self.prev_status_required = time.time()

    def __del__(self):
        self.wait()

    def status_request(self):
        now = time.time()
        if(self.prev_status_required + 0.2 < now):
            self.serial.write(("?").encode("ascii"))
            self.prev_status_required = now

    def parse_status(self, status):
        mpos_idx = status.find("MPos:")
        mpos_str = status[mpos_idx+5:].split("|")[0].split(",")
        mpos = [float(i) for i in mpos_str]
        if(mpos[0] == mpos[2] and mpos[1] == mpos[3]):
            mpos[3] += 0.001
        self._machine.set_position(mpos)

    def play_command_list(self):
        on_board_available_space = 128
        sent_sizes = queue.Queue()
        while(not self.command_list.empty()):
            command = self.command_list.get()
            command_size = len(command)

            #wait for board to free enough buffer space
            while(command_size > on_board_available_space):
                self.status_request()
                result = self.serial.readline().decode("ascii")
                if(result == 'ok\r\n'):
                    on_board_available_space += sent_sizes.get()
                elif(result != ''): #TODO handle grbl errors
                    if(result[0] == "<"):
                        self.parse_status(result)
                    else:
                        pass #error ?

            # send new command
            self.serial.write(command.encode("ascii"))
            on_board_available_space -= command_size
            sent_sizes.put(command_size)

            self.status_request()

    def play(self, gcode):
        if(not self.play_request):
            for command in gcode:
                self.command_list.put(command)
            self.play_request = True
        else:
            print("Error : program already running")

    def run(self):
        self.serial = serial.Serial(self.port, 115200, timeout=2.0)

        crlf = self.serial.readline()
        prompt = self.serial.readline().decode("ascii")
        if(prompt[:4] != "Grbl"):
            print("Error : Grbl not responding.")
            return

        self.serial.timeout = 0.1

        while(True):
            if(self.play_request):
                self.play_command_list()
                self.play_request = False;

            self.status_request()
            result = self.serial.readline().decode("ascii")
            if(result != ''):
                if(result[0] == "<"):
                    self.parse_status(result)
                else:
                    pass #error ?

class ControlWidget(QtGui.QWidget):

    def __init__(self, connected_airfoils, machine):
        super().__init__()

        self._connected_airfoils = connected_airfoils

        self.serial_thread = SerialThread(machine)

        layout = QtGui.QGridLayout()
        play_btn = QtGui.QPushButton("play")
        play_btn.clicked.connect(self.on_play)
        stop_btn = QtGui.QPushButton("stop")
        home_btn = QtGui.QPushButton("home")
        connect_btn = QtGui.QPushButton("connect")
        connect_btn.clicked.connect(self.on_connect)

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

        lead_spbox = QtGui.QDoubleSpinBox()
        lead_spbox.setRange(0, 1000)
        lead_spbox.setValue(10)
        lead_spbox.setPrefix("Lead in/out : ")
        lead_spbox.setSuffix("mm")

        layout.addWidget(self.feed_spbox, 0, 0)
        layout.addWidget(lead_spbox, 1, 0)
        layout.addWidget(home_btn, 2, 0)
        layout.addWidget(play_btn, 3, 0)
        layout.addWidget(stop_btn, 4, 0)
        layout.addWidget(connect_btn, 5, 0)
        layout.addWidget(self.serial_text_item, 0, 1, 5, 1)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 5)
        layout.addWidget(self.port_box, 0, 6)
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

    def on_connect(self):
        self.serial_thread.port = self.port_box.currentText()
        self.serial_thread.start()

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
