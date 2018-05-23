#!/usr/bin/python3
# -*- coding: utf-8 -*-
from math import *
from PyQt5 import QtCore
import numpy as np
import sys
import os

################################################################################
class Airfoil(QtCore.QObject):
    ###############################################
    data_changed = QtCore.pyqtSignal()

    def __init__(self, filename = None):
        super().__init__()
        self.loaded = False
        self.name = ""
        self.r = 0
        self.s = 100
        self.t = [0, 0]
        self.d = 0
        self.lead_in_out = 10
        self.orig_data = self.trans_data = np.array([[],[]])
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
        self.orig_data = np.array([x, y])

        # normalize x between 0 and 1
        min_x = self.orig_data[0].min()
        max_x = self.orig_data[0].max()
        self.orig_data[0] -= min_x
        if(max_x == min_x):
            self.loaded = False
            return
        self.orig_data /= (max_x - min_x)

        if(not self.__compute_leading_edge()):
            self.orig_data = self.trans_data = np.array([[],[]])
            self.loaded = False
            return

        self.orig_data[0] = -self.orig_data[0]
        self.loaded = True
        self.name = os.path.splitext(os.path.basename(filename))[0]
        self.__apply_transform()
    ###############################################
    def __str__(self):
        return str(self.trans_data)
    ###############################################
    def rotate(self, rotation):
        self.r = rotation
        self.__apply_transform()
    ###############################################
    def scale(self, scale):
        self.s = scale
        self.__apply_transform()
    ###############################################
    def translate_x(self, translation_x):
        self.t[0] = translation_x
        self.__apply_transform()
    ###############################################
    def translate_y(self, translation_y):
        self.t[1] = translation_y
        self.__apply_transform()
    ###############################################
    def dilate(self, radius):
        self.d = radius
        self.__apply_transform()
    ###############################################
    def set_lead(self, lead):
        self.lead_in_out = lead
        self.__apply_transform()
    ###############################################
    def __apply_dilate(self, radius):
        x = self.orig_data[0]
        y = self.orig_data[1]

        x = np.insert(x, 0, 2*x[0]-x[1])
        y = np.insert(y, 0, 2*y[0]-y[1])

        x = np.append(x, 2*x[-1:]-x[-2:-1])
        y = np.append(y, 2*y[-1:]-y[-2:-1])

        angle_a = np.arctan2(y[:-1]-y[1:], x[:-1]-x[1:])[:-1] # 1 to end-1
        angle_b = np.arctan2(y[1:]-y[:-1], x[1:]-x[:-1])[1:] # 1 to end-1
        diff = (angle_b - angle_a + 2*pi)%(2*pi)

        self.dilate_data = np.stack((
            x[1:-1] - radius * np.cos(angle_a + diff/2),
            y[1:-1] - radius * np.sin(angle_a + diff/2)))
    ###############################################
    def __refresh_transform_mat(self):
        rrad = self.r / 180 * pi
        self.tr_mat = np.array([[self.s*cos(rrad), -self.s*sin(rrad), self.t[0]+self.s],
                        [self.s*sin(rrad), self.s*cos(rrad), self.t[1]       ],
                        [0,                 0,                1               ]])
    ###############################################
    def __apply_transform(self):
        if(not self.loaded):
            return

        self.__apply_dilate(self.d / self.s)
        self.__refresh_transform_mat()

        z_padded = np.pad(self.dilate_data, ((0, 1), (0, 0)), 'constant', constant_values=1)
        self.trans_data = np.dot(self.tr_mat, z_padded)[:-1]

        p1 = (self.trans_data[0][0], self.trans_data[1][0])
        p2 = (self.trans_data[0][1], self.trans_data[1][1])
        pm1 = (self.trans_data[0][-1], self.trans_data[1][-1])
        pm2 = (self.trans_data[0][-2], self.trans_data[1][-2])

        p1p2_len = sqrt( (p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 )
        pm1pm2_len = sqrt( (pm2[0] - pm1[0])**2 + (pm2[1] - pm1[1])**2 )

        self.start = (p1[0] - self.lead_in_out * (p2[0] - p1[0]) / p1p2_len, p1[1] - self.lead_in_out * (p2[1] - p1[1]) / p1p2_len)
        self.end = (pm1[0] - self.lead_in_out * (pm2[0] - pm1[0]) / pm1pm2_len, pm1[1] - self.lead_in_out * (pm2[1] - pm1[1]) / pm1pm2_len)

        self.data_changed.emit()
    ###############################################
    def get_interpolated_point_list(self, degree_list):
        if(not self.loaded):
            return

        it_point_list = np.array([[],[]])
        for degree in degree_list:
            w = self.get_interpolated_point(degree)
            it_point_list = np.append(it_point_list, [[w[0]], [w[1]]], axis=1)
        z_padded = np.pad(it_point_list, ((0, 1), (0, 0)), 'constant', constant_values=1)
        it_point_list = np.dot(self.tr_mat, z_padded)[:-1]

        return it_point_list
    ###############################################
    def get_degree_list(self):
        if(not self.loaded):
            return

        degree_list = list()
        min_x = self.dilate_data[0][0]
        max_x = self.dilate_data[0][self.leading_edge_idx]
        for i in self.dilate_data[0][:self.leading_edge_idx]:
            degree_list.append((i - min_x) / (max_x - min_x) / 2)

        degree_list.append(0.5)

        min_x = self.dilate_data[0][-1]
        for i in self.dilate_data[0][self.leading_edge_idx+1:]:
            degree_list.append((1-((i - min_x) / (max_x - min_x))) / 2 + 0.5)

        return degree_list
    ###############################################
    def __compute_leading_edge(self):
        edge_candidates = np.where(self.orig_data[0] == 0)[0]
        self.leading_edge_idx = None
        if(len(edge_candidates) == 0):
            print("Error : No candidate for leading edge, normalization could have failed.")
        elif(len(edge_candidates) == 1):
            self.leading_edge_idx = edge_candidates[0]
        elif(len(edge_candidates) == 2):
            if(edge_candidates[0] + 1 != edge_candidates[1]):
                print("Error : There exist two leading edge that are not contiguous.")
            else:
                ys = np.take(self.orig_data[1], edge_candidates)
                middle_y = (ys[0] + ys[1]) / 2
                self.orig_data = np.insert(self.orig_data, edge_candidates[1], [0, middle_y], axis=1)
                self.leading_edge_idx = edge_candidates[1]
        else:
            print("Error : More than two leading edge candidates.")

        return self.leading_edge_idx is not None
    ###############################################
    def get_interpolated_point(self, degree):
        if degree <= 0.0 :
            return np.take(self.dilate_data, 0, axis=1)

        if degree == 0.5 :
            return np.take(self.dilate_data, self.leading_edge_idx, axis=1)

        if degree >= 1.0 :
            return np.take(self.dilate_data, -1, axis=1)

        if degree < 0.5 :
            min_x = self.dilate_data[0][0]
            max_x = self.dilate_data[0][self.leading_edge_idx]
            degree_scaled = (max_x - min_x)*degree*2 + min_x

            next_idx = np.argmax(self.dilate_data[0] >= degree_scaled)
            prev_idx = next_idx - 1

            prev_p = np.take(self.dilate_data, prev_idx, axis=1)
            next_p = np.take(self.dilate_data, next_idx, axis=1)

            side_gap = next_p[0] - prev_p[0]
            prev_gap = degree_scaled - prev_p[0]
            return prev_p + ((next_p - prev_p) * prev_gap / side_gap)
        else:
            min_x = self.dilate_data[0][-1]
            max_x = self.dilate_data[0][self.leading_edge_idx]
            degree_scaled = (max_x - min_x)*(1-degree)*2 + min_x

            next_idx = np.argmax(self.dilate_data[0][self.leading_edge_idx:] <= degree_scaled) + self.leading_edge_idx
            prev_idx = next_idx - 1

            prev_p = np.take(self.dilate_data, prev_idx, axis=1)
            next_p = np.take(self.dilate_data, next_idx, axis=1)

            side_gap = prev_p[0] - next_p[0]
            prev_gap = prev_p[0] - degree_scaled
            return prev_p + ((next_p - prev_p) * prev_gap / side_gap)
################################################################################
