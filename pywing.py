#!/usr/bin/env python
import matplotlib.pyplot as plt
from math import *
import numpy as np
import sys

################################################################################
class Airfoil:
    ###############################################
    def __init__(self, filename = None):
        if filename == None :
            self.points = np.array(float)
        else:
            data = list()
            fp = open(filename)
            line = ' '
            while line != "":
                line = fp.readline()
                words = line.split()
                try:
                    if len(words) == 2:
                        x = float(words[0])
                        y = float(words[1])
                        data.append([x,y])
                except ValueError:
                    pass
            self.points = np.array(data, float)
    ###############################################
    def __str__(self):
        return str(self.points)
    ###############################################
    def __mul__(self, factor):
        self.points *= factor
        return self
    ###############################################
    __rmul__ = __mul__
    ###############################################
    def dilate(self, radius):
        data2 = list()
        #first contour point
        [x,y]   = self.points[0]
        [xb,yb] = self.points[1]
        angle_b = atan2(yb-y, xb-x)
        data2.append([x + radius*cos(angle_b-pi/2),y + radius*sin(angle_b-pi/2)])

        #contour
        for i in xrange(1, len(self.points)-1):
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
    def plot(self):
        xl = list()
        yl = list()
        for [x,y] in self.points:
            xl.append(x)
            yl.append(y)
        plt.plot(xl, yl)
    ###############################################
    def normalize(self):
        xmin = xmax = self.points[0][0]
        for [x,y] in self.points:
            if x < xmin:
                xmin = x
            if x > xmax:
                xmax = x

        for e in self.points:
            e[0] -= xmin
            e /= (xmax - xmin)
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
            print "Cannot determine leading edge, multiple canditates."
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
        for i in xrange(4):
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

root = Airfoil("/home/lucas/fuselage.dat")
root.normalize()

tip = Airfoil("/home/lucas/fuselage.dat")
tip.normalize()

length = 408
root *= 408
tip  *= 408

# tip.points = tip.points + [80.0, 0.0]

root.dilate(1.0)
tip.dilate(1.1)
# root.points -= 20


fusion = root.merge(tip)
fusion[0] = length - fusion[0] + 2
fusion[2] = length - fusion[2] + 2
fusion[1] += 30
fusion[3] += 30
for i in xrange(len(fusion[0])):
    plt.plot([fusion[0][i], fusion[2][i]], [fusion[1][i], fusion[3][i]])

coeff_a_up = (fusion[1][1]-fusion[1][0])/(fusion[0][1]-fusion[0][0])
plt.plot([0, fusion[0][0]], [fusion[1][0]-(fusion[0][0]*coeff_a_up),fusion[1][0]])

coeff_b_up = (fusion[3][1]-fusion[3][0])/(fusion[2][1]-fusion[2][0])
plt.plot([0, fusion[2][0]], [fusion[3][0]-(fusion[2][0]*coeff_b_up),fusion[3][0]])

# print "G01 X0.000 Y%.3f U0.000 V%.3f F200" %(fusion[1][0]-(fusion[0][0]*coeff_a_up), fusion[3][0]-(fusion[2][0]*coeff_b_up))
print "G01 X0.000 Y%.3f U0.000 V%.3f F200" %(fusion[3][0]-(fusion[2][0]*coeff_b_up), fusion[1][0]-(fusion[0][0]*coeff_a_up))

for i in xrange(len(fusion[0])):
    print "G01 X%.3f Y%.3f U%.3f V%.3f" %(fusion[2][i], fusion[3][i], fusion[0][i], fusion[1][i])
    # print "G01 X%.3f Y%.3f U%.3f V%.3f" %(fusion[0][i], fusion[1][i], fusion[2][i], fusion[3][i])

last = len(fusion[0]) - 1
coeff_a_down = (fusion[1][last-1]-fusion[1][last])/(fusion[0][last-1]-fusion[0][last])
plt.plot([0, fusion[0][last]], [fusion[1][last]-(fusion[0][last]*coeff_a_down),fusion[1][last]])

coeff_b_down = (fusion[3][last-1]-fusion[3][last])/(fusion[2][last-1]-fusion[2][last])
plt.plot([0, fusion[2][last]], [fusion[3][last]-(fusion[2][last]*coeff_a_down),fusion[3][last]])

# print "G01 X0.000 Y%.3f U0.000 V%.3f" %(fusion[1][last]-(fusion[0][last]*coeff_a_down), fusion[3][last]-(fusion[2][last]*coeff_a_down))
print "G01 X0.000 Y%.3f U0.000 V%.3f" %(fusion[3][last]-(fusion[2][last]*coeff_a_down), fusion[1][last]-(fusion[0][last]*coeff_a_down))

root.points = [length+10, 0] + root.points*[-1,1]
tip.points = [length+10, 0] + tip.points*[-1,1]
root.plot()
tip.plot()

plt.axis([0, length+20, -length/3, length/3])
# plt.axis([-0.1, 1.1, -1, 1])
plt.show()
