#!/usr/bin/env python3
import numpy as np
import svgpathtools, sys, os
from pathgenerator import *

class SVGLoader():
    def load(filename):
        paths = svgpathtools.svg2paths(filename, convert_lines_to_paths=True, convert_polylines_to_paths=True, convert_polygons_to_paths=True, return_svg_attributes=False)
        item_list = list()
        for i in paths[0][0]:
            # print(isinstance, i, svgpathtools.path.CubicBezier)
            # if isinstance(i, svgpathtools.path.CubicBezier):
            #     print("Cb",svgpathtools.real(i.point(0.0)),svgpathtools.imag(i.point(0.0)), svgpathtools.real(i.point(1.0)),svgpathtools.imag(i.point(1.0)))
            # elif isinstance(i, svgpathtools.path.Line):
            #     print("L",svgpathtools.real(i.point(0.0)),svgpathtools.imag(i.point(0.0)), svgpathtools.real(i.point(1.0)),svgpathtools.imag(i.point(1.0)))
            item_list.append(Line((svgpathtools.real(i.point(0.0)),svgpathtools.imag(i.point(0.0))), (svgpathtools.real(i.point(1.0)),svgpathtools.imag(i.point(1.0)))))

        gen = PathGenerator()
        gen.items = item_list
        return gen

        dwg = ezdxf.readfile(filename)
        msp = dwg.modelspace()
        item_list = list()
        ends = list()
        for line in msp.query('LINE'):
            item_list.append(Line(line.dxf.start, line.dxf.end))
            ends += [line.dxf.start, line.dxf.end]
        for arc in msp.query('ARC'):
            item = Arc(arc.dxf.center, arc.dxf.radius, arc.dxf.start_angle * math.pi / 180, arc.dxf.end_angle * math.pi / 180, True)
            ends += [item.start, item.end]
            item_list.append(item)
        for polyline in msp.query('LWPOLYLINE'):
            # (x, y, [start_width, [end_width, [bulge]]])
            # TODO handle bulge
            points = polyline.get_points()
            prev_p = points[0]
            for p in points[1:]:
                item_list.append(Line(prev_p[:2], p[:2]))
                prev_p = p
            ends += [points[0][:2], points[-1][:2]]

        if not item_list:
            raise Exception('Path is empty')
        if len(item_list) == 1:
            pass
        else:
            edge_list = np.empty((0,2), int)
            for n in range(len(ends)):
                for m in range(n+1, len(ends)):
                    if(abs(ends[n][0] - ends[m][0]) < epsilon and abs(ends[n][1] - ends[m][1]) < epsilon):
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
