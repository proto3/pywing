import numpy as np
import svgpathtools as svgpt
import sys, os
from pathgenerator import *

def bezier_to_lines(bezier):
    epsilon = 1e-2

    t = np.linspace(0.0, 1.0, num=4).reshape(4,1)
    points = (bezier[0] * (1-t)**3 +
              bezier[1] * 3 * t * (1-t)**2 +
              bezier[2] * 3 * t**2 * (1-t) +
              bezier[3] * t**3)

    i = 1
    curve_data = np.hstack((t, points)).transpose()
    while i < np.size(curve_data, 1):
        t_mid = (curve_data[0][i] + curve_data[0][i-1]) / 2

        a = curve_data[1:,i-1]
        b = curve_data[1:,i]
        c = (bezier[0] * (1-t_mid)**3 +
             bezier[1] * 3 * t_mid * (1-t_mid)**2 +
             bezier[2] * 3 * t_mid**2 * (1-t_mid) +
             bezier[3] * t_mid**3)
        ab = b - a
        ac = c - a
        ac_proj_in_ab = np.sum(ab*ac, axis=0) / np.sum(np.square(ab), axis=0)
        dist_to_curve = np.linalg.norm(ac_proj_in_ab * ab - ac, axis=0)

        if dist_to_curve > epsilon:
            curve_data = np.insert(curve_data, i, [t_mid, c[0], c[1]], axis=1)
        else:
            i += 1

    return curve_data[1:]

class SVGLoader():
    px_per_inch = 96
    mm_per_inch = 25.4
    px_per_mm = px_per_inch / mm_per_inch

    def load(filename):
        svg_items = svgpt.svg2paths(filename, convert_lines_to_paths=True, convert_polylines_to_paths=True, convert_polygons_to_paths=True, return_svg_attributes=False)

        path = np.array([[],[]])
        for i in svg_items[0][0]:
            if isinstance(i, svgpt.path.CubicBezier):
                cubic_bezier = np.array([[svgpt.real(i.start),    svgpt.imag(i.start)],
                                         [svgpt.real(i.control1), svgpt.imag(i.control1)],
                                         [svgpt.real(i.control2), svgpt.imag(i.control2)],
                                         [svgpt.real(i.end),      svgpt.imag(i.end)]])
                cubic_bezier /= SVGLoader.px_per_mm

                points = bezier_to_lines(cubic_bezier)
                points = np.repeat(points, 2, axis=1)[:,1:-1]
                path = np.column_stack((path, points))

            elif isinstance(i, svgpt.path.Line):
                line = np.array([[svgpt.real(i.point(0.0)), svgpt.real(i.point(1.0))],
                                 [svgpt.imag(i.point(0.0)), svgpt.imag(i.point(1.0))]])
                line /= SVGLoader.px_per_mm

                path = np.column_stack((path, line))

        # lay path on 0,0
        path -= np.amin(path, axis=1).reshape(2,1)

        # reverse Y axis
        path[1] = np.amax(path, axis=1)[1] - path[1]

        item_list = list()
        for i in range(0, np.size(path, 1), 2):
            item_list.append(Line(path[:,i], path[:,i+1]))
        return PathGenerator(item_list)
