#!/usr/bin/python3
# from PyQt5 import QtCore
import numpy as np
import math

class Path():
# class Path(QtCore.QObject):
    # update = QtCore.pyqtSignal()
    # reset = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self.initial_path = np.array([[],[]])
        self.final_path   = np.array([[],[]])
        self.s = 1.0        # Scale
        self.r = 0.0        # Rotation
        self.t = [0.0, 0.0] # Translation
        self.k = 0.0        # Kerf width
        self.l = 10.0       # Lead in/out length

        self.tr_mat   = np.array([])
        self.lead_in  = np.array([])
        self.lead_out = np.array([])

    def __str__(self):
        return str(self.final_path)

    def export_tuple(self):
        return (self.s, self.k, self.initial_path)

    def import_tuple(self, tuple):
        self.s, self.k, self.initial_path = tuple
        self._apply_transform()
        # self.reset.emit()

    def get_path(self):
        return self.final_path

    def get_boundaries(self):
        # return [xmin, ymin, xmax, ymax]
        return np.concatenate((np.amin(self.final_path, axis=1), np.amax(self.final_path, axis=1)))

    def scale(self, s):
        self.s = s
        self._apply_transform()

    def rotate(self, r):
        self.r = r
        self._apply_transform()

    def translate_x(self, t):
        self.t[0] = t
        self._apply_transform()

    def translate_y(self, t):
        self.t[1] = t
        self._apply_transform()

    def set_kerf_width(self, k):
        self.k = k
        self._apply_transform()

    def set_lead_size(self, l):
        self.l = l
        self._apply_transform()

    def _apply_transform(self):
        if self.initial_path.size == 0:
            return

        # apply kerf width correction
        self._apply_kerf()

        r_rad = self.r / 180 * math.pi
        a = self.s * math.cos(r_rad)
        b = self.s * math.sin(r_rad)

        #prepare transform matrix
        self.tr_mat = np.array([[a,-b, self.t[0]],
                                [b, a, self.t[1]],
                                [0, 0,         1]])

        # apply matrix
        self.final_path = np.dot(self.tr_mat, np.insert(self.kerf_path, 2, 1.0, axis=0))[:-1]

        # append lead in and out to path
        self.lead_in  = self._lead_next(self.final_path[:, 1], self.final_path[:, 0])
        self.lead_out = self._lead_next(self.final_path[:,-2], self.final_path[:,-1])
        self.final_path = np.column_stack((self.lead_in, self.final_path, self.lead_out))

        # self.update.emit()

    def _apply_kerf(self):
        tmp = self.initial_path
        x, y = np.column_stack((2*tmp[:,0]-tmp[:,1], tmp, 2*tmp[:,-1]-tmp[:,-2]))

        angle = np.arctan2(y[1:]-y[:-1], x[1:]-x[:-1])
        diff = (angle[:-1] - angle[1:] + math.pi)%(2*math.pi)

        radius = self.k / self.s
        self.kerf_path = np.stack((
            x[1:-1] - radius * np.cos(angle[1:] + math.pi + diff/2),
            y[1:-1] - radius * np.sin(angle[1:] + math.pi + diff/2)))

    def _lead_next(self, a, b):
        # return c in | a---b ---- c | where c is the lead entry
        return b + (b-a) * self.l / np.linalg.norm(b-a)
