#!/usr/bin/python3
# -*- coding: utf-8 -*-
from math import *
from PyQt5 import QtCore
import numpy as np
import sys, os

class PathGenerator(QtCore.QObject):
    data_changed = QtCore.pyqtSignal()
    control_changed = QtCore.pyqtSignal()

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

    def export_data(self):
        return (self.s, self.k, self.initial_path)

    def import_data(self, data):
        self.s, self.k, self.initial_path = data
        self._apply_transform()
        self.control_changed.emit()

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

        r_rad = self.r / 180 * pi
        a = self.s * cos(r_rad)
        b = self.s * sin(r_rad)

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

        self.data_changed.emit()

    def _apply_kerf(self):
        tmp = self.initial_path
        x, y = np.column_stack((2*tmp[:,0]-tmp[:,1], tmp, 2*tmp[:,-1]-tmp[:,-2]))

        angle = np.arctan2(y[1:]-y[:-1], x[1:]-x[:-1])
        diff = (angle[:-1] - angle[1:] + pi)%(2*pi)

        radius = self.k / self.s
        self.kerf_path = np.stack((
            x[1:-1] - radius * np.cos(angle[1:] + pi + diff/2),
            y[1:-1] - radius * np.sin(angle[1:] + pi + diff/2)))

    def _lead_next(self, a, b):
        # return c in | a---b ---- c | where c is the lead entry
        return b + (b-a) * self.l / np.linalg.norm(b-a)
################################################################################
class AirfoilGenerator(PathGenerator):
    ###############################################
    def __init__(self, filename = None):
        super().__init__()
        self.loaded = False
        self.name = ""
        self.s = 100.0
        self.leading_edge_idx = 0
        if filename is not None :
            self.load(filename)
    ###############################################
    def load(self, filename):
        x = list()
        y = list()
        fp = open(filename)
        line = ' '
        while line != "":
            line = fp.readline()
            words = line.split()
            try:
                if len(words) == 2:
                    a, b = (float(words[0]), float(words[1]))
                    x.append(a)
                    y.append(b)
            except ValueError:
                pass
        self.initial_path = np.array([x, y])

        # normalize x between 0 and 1
        min_x = self.initial_path[0].min()
        max_x = self.initial_path[0].max()
        self.initial_path[0] -= min_x
        if(max_x == min_x):
            self.loaded = False
            return
        self.initial_path /= (max_x - min_x)

        if(not self.__compute_leading_edge()):
            self.initial_path = self.final_path = np.array([[],[]])
            self.loaded = False
            return

        self.initial_path[0] = -self.initial_path[0]
        self.loaded = True
        self.name = os.path.splitext(os.path.basename(filename))[0]
        self._apply_transform()
    ###############################################
    def export_data(self):
        return super(AirfoilGenerator, self).export_data() , self.name, self.loaded, self.leading_edge_idx
    ###############################################
    def import_data(self, data):
        self.name, self.loaded, self.leading_edge_idx = data[1:]
        super(AirfoilGenerator, self).import_data(data[0])
    ###############################################
    def get_interpolated_points(self, degree_list):
        if(not self.loaded):
            return

        itpl_points = np.array([[],[]])
        for degree in degree_list:
            w = self.get_interpolated_point(degree)
            itpl_points = np.append(itpl_points, [[w[0]], [w[1]]], axis=1)
        z_padded = np.pad(itpl_points, ((0, 1), (0, 0)), 'constant', constant_values=1)
        itpl_points = np.dot(self.tr_mat, z_padded)[:-1]

        return np.column_stack((self.lead_in, itpl_points, self.lead_out))
    ###############################################
    def get_degree_list(self):
        if(not self.loaded):
            return

        degree_list = list()
        min_x = self.kerf_path[0][0]
        max_x = self.kerf_path[0][self.leading_edge_idx]
        for i in self.kerf_path[0][:self.leading_edge_idx]:
            degree_list.append((i - min_x) / (max_x - min_x) / 2)

        degree_list.append(0.5)

        min_x = self.kerf_path[0][-1]
        for i in self.kerf_path[0][self.leading_edge_idx+1:]:
            degree_list.append((1-((i - min_x) / (max_x - min_x))) / 2 + 0.5)

        return degree_list
    ###############################################
    def __compute_leading_edge(self):
        edge_candidates = np.where(self.initial_path[0] == 0)[0]
        self.leading_edge_idx = None
        if(len(edge_candidates) == 0):
            print("Error : No candidate for leading edge, normalization could have failed.")
        elif(len(edge_candidates) == 1):
            self.leading_edge_idx = edge_candidates[0]
        elif(len(edge_candidates) == 2):
            if(edge_candidates[0] + 1 != edge_candidates[1]):
                print("Error : There exist two leading edge that are not contiguous.")
            else:
                ys = np.take(self.initial_path[1], edge_candidates)
                middle_y = (ys[0] + ys[1]) / 2
                self.initial_path = np.insert(self.initial_path, edge_candidates[1], [0, middle_y], axis=1)
                self.leading_edge_idx = edge_candidates[1]
        else:
            print("Error : More than two leading edge candidates.")

        return self.leading_edge_idx is not None
    ###############################################
    def get_interpolated_point(self, degree):
        if degree <= 0.0 :
            return np.take(self.kerf_path, 0, axis=1)

        if degree == 0.5 :
            return np.take(self.kerf_path, self.leading_edge_idx, axis=1)

        if degree >= 1.0 :
            return np.take(self.kerf_path, -1, axis=1)

        if degree < 0.5 :
            min_x = self.kerf_path[0][0]
            max_x = self.kerf_path[0][self.leading_edge_idx]
            degree_scaled = (max_x - min_x)*degree*2 + min_x

            next_idx = np.argmax(self.kerf_path[0] >= degree_scaled)
            prev_idx = next_idx - 1

            prev_p = np.take(self.kerf_path, prev_idx, axis=1)
            next_p = np.take(self.kerf_path, next_idx, axis=1)

            side_gap = next_p[0] - prev_p[0]
            prev_gap = degree_scaled - prev_p[0]
            return prev_p + ((next_p - prev_p) * prev_gap / side_gap)
        else:
            min_x = self.kerf_path[0][-1]
            max_x = self.kerf_path[0][self.leading_edge_idx]
            degree_scaled = (max_x - min_x)*(1-degree)*2 + min_x

            next_idx = np.argmax(self.kerf_path[0][self.leading_edge_idx:] <= degree_scaled) + self.leading_edge_idx
            prev_idx = next_idx - 1

            prev_p = np.take(self.kerf_path, prev_idx, axis=1)
            next_p = np.take(self.kerf_path, next_idx, axis=1)

            side_gap = prev_p[0] - next_p[0]
            prev_gap = prev_p[0] - degree_scaled
            return prev_p + ((next_p - prev_p) * prev_gap / side_gap)

################################################################################
