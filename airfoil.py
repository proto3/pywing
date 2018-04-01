#!/usr/bin/python3
# -*- coding: utf-8 -*-
from math import *
import numpy as np
import sys

################################################################################
class Airfoil:
    ###############################################
    def __init__(self, filename = None):
        self.r = 0
        self.s = 100
        self.t = [0, 0]
        if filename == None :
            self.raw_x = self.x = np.array([])
            self.raw_y = self.y = np.array([])
        else:
            self.load(filename)
    ###############################################
    def load(self, filename):
        xl = list()
        yl = list()
        fp = open(filename)
        line = ' '
        while line != "":
            line = fp.readline()
            words = line.split()
            try:
                if len(words) == 2:
                    xl.append(float(words[0]))
                    yl.append(float(words[1]))
            except ValueError:
                pass
        self.raw_x = self.x = np.array(xl)
        self.raw_y = self.y = np.array(yl)
        self.__apply_transform()
    ###############################################
    def __str__(self):
        return str((self.x, self.y))
    ###############################################
    def rotate(self, rotation):
        self.r = rotation
        self.__apply_transform()
    ###############################################
    def scale(self, scale):
        self.s = scale
        self.__apply_transform()
    ###############################################
    def translate(self, translation):
        self.t = translation
        self.__apply_transform()
    ###############################################
    def __apply_transform(self):
        rrad = self.r / 180 * pi
        mat = np.array([[-self.s*cos(rrad), self.s*sin(rrad), self.t[0]+self.s],
                        [self.s*sin(rrad), self.s*cos(rrad), self.t[1]       ],
                        [0,                 0,                1               ]])
        (self.x, self.y, _) = np.dot(mat, [self.raw_x, self.raw_y, np.ones((len(self.raw_x)))])
    ###############################################
    def dilate(self, radius):
        data2 = list()
        #first contour point
        [x,y]   = self.points[0]
        [xb,yb] = self.points[1]
        angle_b = atan2(yb-y, xb-x)
        data2.append([x + radius*cos(angle_b-pi/2),y + radius*sin(angle_b-pi/2)])

        #contour
        for i in range(1, len(self.points)-1):
            [xa,ya] = self.points[i-1]
            [x,y]   = self.points[i]
            [xb,yb] = self.points[i+1]
            angle_a = atan2(ya-y, xa-x)
            angle_b = atan2(yb-y, xb-x)
            if angle_a < angle_b :
                diff = angle_b - angle_a
            else:
                diff = (2*pi) - angle_a + angle_b
            data2.append([x + (radius*cos(angle_a+diff/2)),y + (radius*sin(angle_a+diff/2))])

        #last contour point
        [xa,ya] = self.points[len(self.points)-2]
        [x,y]   = self.points[len(self.points)-1]
        angle_a = atan2(ya-y, xa-x)
        data2.append([x + radius*cos(angle_b-pi/2),y + radius*sin(angle_b-pi/2)])
        self.points = np.array(data2, float)
    ###############################################
    def length(self):
        xmin = xmax = self.points[0][0]
        for [x,y] in self.points:
            if x < xmin:
                xmin = x
            if x > xmax:
                xmax = x
        return xmax - xmin
    ###############################################
    def leading_edge_idx(self):
        xmin_idx = 0

        for i, [x,y] in enumerate(self.points):
            if x < self.points[xmin_idx][0] :
                xmin_idx = i

        xmin_count = 0
        for [x,y] in self.points:
            if x == self.points[xmin_idx][0] :
                xmin_count += 1

        if xmin_count > 1 :
            print("Cannot determine leading edge, multiple canditates.")
            sys.exit(0)

        return xmin_idx
    ###############################################
    def get_point(self, val):
        if val <= 0.0 :
            return self.points[0]

        if val == 0.5 :
            return self.points[self.leading_edge_idx()]

        if val >= 1.0 :
            return self.points[len(self.points)-1]

        if val < 0.5 :
            x = 2 * (0.5 - val) * self.length()
            x += self.points[self.leading_edge_idx()][0]
            i = 0
            while self.points[i][0] > x:
                i += 1

            next_p = self.points[i]
            prev_p = self.points[i-1]
            next_dist = x - next_p[0]
            prev_dist = prev_p[0] - x

            return (next_p * prev_dist + prev_p * next_dist) / (prev_dist + next_dist)
        else:
            x = 2 * (val - 0.5) * self.length()
            x += self.points[self.leading_edge_idx()][0]
            i = self.leading_edge_idx()
            while self.points[i][0] < x:
                i += 1

            next_p = self.points[i]
            prev_p = self.points[i-1]
            next_dist = next_p[0] - x
            prev_dist = x - prev_p[0]

            return (next_p * prev_dist + prev_p * next_dist) / (prev_dist + next_dist)
    ###############################################
    def get_normalized_pos(self, idx):
        lead = self.leading_edge_idx()
        length = self.length()
        if idx <= 0 :
            return 0.0
        if idx == lead :
            return 0.5
        if idx >= len(self.points) - 1 :
            return 1.0

        if idx < lead :
            return (length - self.points[idx][0] + self.points[lead][0]) / (2.0 * length)
        else:
            return (self.points[idx][0] - self.points[lead][0]) / (2.0 * length) + 0.5
    ###############################################
    def merge(self, airfoil):
        counter_a = 1
        counter_b = 1
        output = list()
        for i in range(4):
            output.append(list())

        output[0].append(self.points[0][0])
        output[1].append(self.points[0][1])
        output[2].append(airfoil.points[0][0])
        output[3].append(airfoil.points[0][1])

        while(counter_a != len(self.points-1) and counter_b != len(airfoil.points-1)):
            npos_a = self.get_normalized_pos(counter_a)
            npos_b = airfoil.get_normalized_pos(counter_b)

            if npos_a == npos_b :
                counter_a += 1
                counter_b += 1
                npos = npos_a
            if npos_a > npos_b :
                counter_a += 1
                npos = npos_a
            else:
                counter_b += 1
                npos = npos_b

            p_a = self.get_point(npos)
            p_b = airfoil.get_point(npos)
            # output.append([p_a, p_b])
            output[0].append(p_a[0])
            output[1].append(p_a[1])
            output[2].append(p_b[0])
            output[3].append(p_b[1])

        return np.array(output, float)
################################################################################
