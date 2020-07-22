import numpy as np
import sys, os
from pathgenerator import *

class AirfoilLoader():
    def load(filename):
        item_list = list()

        fp = open(filename)
        line = ' '
        points = list()
        while line != '':
            line = fp.readline()
            words = line.split()
            try:
                if len(words) == 2:
                    points.append((-float(words[0])*100, float(words[1])*100))
            except ValueError:
                pass

        items = []
        for i in range(len(points)-1):
            items.append(Line(points[i], points[i+1]))

        nmax = 0
        xmax = -1.0
        ymin = -1.0
        ymax = 1.0
        for n, i in enumerate(items):
            if i.start[0] == xmax:
                ymin = min(ymin, i.start[1])
                ymax = max(ymax, i.start[1])
            if i.start[0] > xmax:
                xmax = i.start[0]
                ymin = ymax = i.start[1]
                nmax = n

        leading_edge_y = (ymin + ymax) / 2
        while items[nmax].end[1] > leading_edge_y:
            nmax += 1

        gen = PathGenerator(items)
        leading_edge_deg = gen.degrees()[nmax] + ((items[nmax].start[1] - leading_edge_y)/(items[nmax].start[1] - items[nmax].end[1]))* items[nmax].length() / gen.length()
        gen.sync_points.append(leading_edge_deg)
        return gen
