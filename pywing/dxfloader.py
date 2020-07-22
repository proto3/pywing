import numpy as np
import ezdxf, sys, os
from pathgenerator import *

class DXFLoader():
    def load(filename):
        dwg = ezdxf.readfile(filename)
        msp = dwg.modelspace()
        item_list = list()
        vertex = list()
        for line in msp.query('LINE'):
            item_list.append(Line(line.dxf.start, line.dxf.end))
            vertex += [line.dxf.start, line.dxf.end]
        for arc in msp.query('ARC'):
            item = Arc(arc.dxf.center, arc.dxf.radius, arc.dxf.start_angle * math.pi / 180, arc.dxf.end_angle * math.pi / 180, True)
            vertex += [item.start, item.end]
            item_list.append(item)
        for polyline in msp.query('LWPOLYLINE'):
            # (x, y, [start_width, [end_width, [bulge]]])
            # TODO handle bulge
            points = polyline.get_points()
            prev_p = points[0]
            for p in points[1:]:
                item_list.append(Line(prev_p[:2], p[:2]))
                prev_p = p
            vertex += [points[0][:2], points[-1][:2]]

        if not item_list:
            raise Exception('Path is empty')
        if len(item_list) == 1:
            pass
        else:
            edge_list = np.empty((0,2), int)
            for n in range(len(vertex)):
                for m in range(n+1, len(vertex)):
                    if(abs(vertex[n][0] - vertex[m][0]) < epsilon and abs(vertex[n][1] - vertex[m][1]) < epsilon):
                        edge_list = np.vstack((edge_list, np.array((n,m))))

            degree = np.bincount(edge_list.flatten()).max()
            if degree > 2:
                raise Exception("Graph is not a path")

        path_gen = PathGenerator(item_list.pop(0))
        add_elt = True
        while add_elt:
            add_elt = False
            for n,i in enumerate(item_list):
                try:
                    path_gen.append(i)
                    item_list.pop(n)
                    add_elt = True
                    break
                except Exception:
                    pass
        path_gen.reverse()
        add_elt = True
        while add_elt:
            add_elt = False
            for n,i in enumerate(item_list):
                try:
                    path_gen.append(i)
                    item_list.pop(n)
                    add_elt = True
                    break
                except Exception:
                    pass

        if len(item_list) > 0:
            raise Exception("Non-connected graph")

        return path_gen
