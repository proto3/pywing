#!/usr/bin/env python3
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
                    points.append((-float(words[0]), float(words[1])))
            except ValueError:
                pass

        items = []
        for i in range(len(points)-1):
            items.append(Line(points[i], points[i+1]))

        return PathGenerator(items)

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

#     def export_tuple(self):
#         return super(AirfoilGenerator, self).export_tuple(), self.name, self.color, self.loaded, self.leading_edge_idx
#
#     def import_tuple(self, tuple):
#         self.name, self.color, self.loaded, self.leading_edge_idx = tuple[1:]
#         super(AirfoilGenerator, self).import_tuple(tuple[0])
#
#     def get_interpolated_points(self, degree_list):
#         if(not self.loaded):
#             return
#
#         itpl_points = np.array([[],[]])
#         for degree in degree_list:
#             w = self.get_interpolated_point(degree)
#             itpl_points = np.append(itpl_points, [[w[0]], [w[1]]], axis=1)
#         z_padded = np.pad(itpl_points, ((0, 1), (0, 0)), 'constant', constant_values=1)
#         itpl_points = np.dot(self.tr_mat, z_padded)[:-1]
#
#         return np.column_stack((self.lead_in, itpl_points, self.lead_out))
#
#     def get_degree_list(self):
#         if(not self.loaded):
#             return
#
#         degree_list = list()
#         min_x = self.kerf_path[0][0]
#         max_x = self.kerf_path[0][self.leading_edge_idx]
#         for i in self.kerf_path[0][:self.leading_edge_idx]:
#             degree_list.append((i - min_x) / (max_x - min_x) / 2)
#
#         degree_list.append(0.5)
#
#         min_x = self.kerf_path[0][-1]
#         for i in self.kerf_path[0][self.leading_edge_idx+1:]:
#             degree_list.append((1-((i - min_x) / (max_x - min_x))) / 2 + 0.5)
#
#         return degree_list
#
#     def __compute_leading_edge(self):
#         edge_candidates = np.where(self.initial_path[0] == 0)[0]
#         self.leading_edge_idx = None
#         if(len(edge_candidates) == 0):
#             print("Error : No candidate for leading edge, normalization could have failed.")
#         elif(len(edge_candidates) == 1):
#             self.leading_edge_idx = edge_candidates[0]
#         elif(len(edge_candidates) == 2):
#             if(edge_candidates[0] + 1 != edge_candidates[1]):
#                 print("Error : There exist two leading edge that are not contiguous.")
#             else:
#                 ys = np.take(self.initial_path[1], edge_candidates)
#                 middle_y = (ys[0] + ys[1]) / 2
#                 self.initial_path = np.insert(self.initial_path, edge_candidates[1], [0, middle_y], axis=1)
#                 self.leading_edge_idx = edge_candidates[1]
#         else:
#             print("Error : More than two leading edge candidates.")
#
#         return self.leading_edge_idx is not None
#
#     def get_interpolated_point(self, degree):
#         if degree <= 0.0 :
#             return np.take(self.kerf_path, 0, axis=1)
#
#         if degree == 0.5 :
#             return np.take(self.kerf_path, self.leading_edge_idx, axis=1)
#
#         if degree >= 1.0 :
#             return np.take(self.kerf_path, -1, axis=1)
#
#         if degree < 0.5 :
#             min_x = self.kerf_path[0][0]
#             max_x = self.kerf_path[0][self.leading_edge_idx]
#             degree_scaled = (max_x - min_x)*degree*2 + min_x
#
#             next_idx = np.argmax(self.kerf_path[0] >= degree_scaled)
#             prev_idx = next_idx - 1
#
#             prev_p = np.take(self.kerf_path, prev_idx, axis=1)
#             next_p = np.take(self.kerf_path, next_idx, axis=1)
#
#             side_gap = next_p[0] - prev_p[0]
#             prev_gap = degree_scaled - prev_p[0]
#             return prev_p + ((next_p - prev_p) * prev_gap / side_gap)
#         else:
#             min_x = self.kerf_path[0][-1]
#             max_x = self.kerf_path[0][self.leading_edge_idx]
#             degree_scaled = (max_x - min_x)*(1-degree)*2 + min_x
#
#             next_idx = np.argmax(self.kerf_path[0][self.leading_edge_idx:] <= degree_scaled) + self.leading_edge_idx
#             prev_idx = next_idx - 1
#
#             prev_p = np.take(self.kerf_path, prev_idx, axis=1)
#             next_p = np.take(self.kerf_path, next_idx, axis=1)
#
#             side_gap = prev_p[0] - next_p[0]
#             prev_gap = prev_p[0] - degree_scaled
#             return prev_p + ((next_p - prev_p) * prev_gap / side_gap)
#
# class AirfoilWidget(QtGui.QWidget):
#     def __init__(self, airfoil):
#         super().__init__()
#         self.airfoil = airfoil
#
#         self.name = QtGui.QLabel(text=self.get_display_name())
#         self.name.setAlignment(Qt.Qt.AlignCenter)
#         self.name.setStyleSheet("color: rgb" + str(self.airfoil.color))
#
#         self.load_btn = QtGui.QPushButton("Load")
#         self.load_btn.clicked.connect(self.on_load)
#
#         self.scale_spbox = QtGui.QDoubleSpinBox()
#         self.scale_spbox.setRange(1, 10000)
#         self.scale_spbox.setValue(self.airfoil.s)
#         self.scale_spbox.setPrefix("S : ")
#         self.scale_spbox.setSuffix("mm")
#         self.scale_spbox.valueChanged.connect(self.on_scale)
#
#         self.kerf_spbox = QtGui.QDoubleSpinBox()
#         self.kerf_spbox.setRange(0, 100)
#         self.kerf_spbox.setValue(self.airfoil.k)
#         self.kerf_spbox.setSingleStep(0.1)
#         self.kerf_spbox.setPrefix("K : ")
#         self.kerf_spbox.setSuffix("mm")
#         self.kerf_spbox.valueChanged.connect(self.on_kerf)
#
#         self.widgets = (self.name, self.load_btn, self.scale_spbox, self.kerf_spbox)
#
#         layout = QtGui.QVBoxLayout()
#         [layout.addWidget(w) for w in self.widgets]
#         layout.addStretch()
#         self.setLayout(layout)
#
#         self.airfoil.reset.connect(self.update)
#
#     def on_load(self):
#         filename, _ = QtGui.QFileDialog.getOpenFileName(self.load_btn.parent(), "Open File", airfoil_data_folder, "Airfoil Files (*.dat *.cor);; All Files (*)")
#         if filename:
#             self.airfoil.load(filename)
#             self.name.setText(self.airfoil.name)
#
#     def on_scale(self):
#         self.airfoil.scale(self.scale_spbox.value())
#
#     def on_kerf(self):
#         self.airfoil.set_kerf_width(self.kerf_spbox.value())
#
#     def update(self):
#         [w.blockSignals(True) for w in self.widgets]
#         self.name.setText(self.get_display_name())
#         self.name.setStyleSheet("color: rgb" + str(self.airfoil.color))
#         self.scale_spbox.setValue(self.airfoil.s)
#         self.kerf_spbox.setValue(self.airfoil.k)
#         [w.blockSignals(False) for w in self.widgets]
#
#     def get_display_name(self):
#         if self.airfoil.loaded:
#             return self.airfoil.name
#         else:
#             return 'No airfoil loaded'
