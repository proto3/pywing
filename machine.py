#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from PyQt5 import QtCore
import time, queue
import serial.tools.list_ports

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
