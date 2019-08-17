#!/usr/bin/env python3
import numpy as np
import ezdxf, sys, os
from pathgenerator import *

class DXFLoader():
    def load(filename):
        dwg = ezdxf.readfile(filename)
        msp = dwg.modelspace()

        item_list = list()
        for e in msp:
            if e.dxftype() == 'LINE':
                item_list.append(Line(e.dxf.start[:2], e.dxf.end[:2]))
            elif e.dxftype() == 'ARC':
                item_list.append(Arc(e.dxf.center[:2], e.dxf.radius, e.dxf.start_angle * math.pi / 180, e.dxf.end_angle * math.pi / 180, True))

        if not item_list:
            raise Exception('Path is empty')
        if len(item_list) == 1:
            pass
        else:
            edge_list = np.empty((0,2), int)
            for n in range(len(item_list)):
                for m in range(n+1, len(item_list)):
                    if(item_list[n].is_connected_to(item_list[m])):
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
