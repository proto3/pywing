#!/usr/bin/env python3
from abc import ABC, abstractmethod
import numpy as np
import math, copy

epsilon = 1e-3

class PathItem(ABC):
    @abstractmethod
    def length(self):
        pass

    @abstractmethod
    def generate(self, reverse=False):
        pass

    @abstractmethod
    def split(self, degree):
        pass

    @abstractmethod
    def reverse(self):
        pass

    @abstractmethod
    def nb_points_hint(self):
        pass

    def get_nb_points(self):
        return self.nb_points

    def set_nb_points(self, nb_points):
        if(nb_points < self.nb_points_hint()):
            raise ValueError('Cannot set nb_points below nb_points_hint')
        self.nb_points = nb_points

    def orient_after(self, other):
        if np.allclose(self.start, other.end, atol=epsilon):
            pass
        elif np.allclose(self.end, other.end, atol=epsilon):
            self.reverse()
        else:
            raise Exception('Cannot orient after PathItem')

    def is_followed_by(self, other):
        return np.allclose(self.end, other.start, atol=epsilon)

    def is_connected_to(self, other):
        return (np.allclose(self.start, other.start, atol=epsilon) or
                np.allclose(self.start, other.end, atol=epsilon) or
                np.allclose(self.end, other.start, atol=epsilon) or
                np.allclose(self.end, other.end, atol=epsilon))

class Line(PathItem):
    def __init__(self, start, end):
        self.start = np.array(start)
        self.end = np.array(end)
        self.nb_points = self.nb_points_hint()

    def __str__(self):
        return 'line: ' + str(self.start) + ' ' + str(self.end) + '\n'

    def length(self):
        return np.linalg.norm(self.start - self.end)

    def generate(self, reverse=False):
        if reverse:
            return np.linspace(self.end, self.start, num=self.nb_points, axis=1)
        else:
            return np.linspace(self.start, self.end, num=self.nb_points, axis=1)

    def split(self, degree):
        splitlen = self.start + (self.end - self.start) * degree
        a = Line(self.start, splitlen)
        b = Line(splitlen, self.end)
        return (a, b)

    def reverse(self):
        self.start, self.end = self.end, self.start

    def nb_points_hint(self):
        return 2

class Arc(PathItem):
    def __init__(self, center, radius, rad_start, rad_end, ccw):
        self.center = np.array(center)
        self.radius = radius
        self.ccw = ccw
        self.rad_start = rad_start
        self.rad_end = rad_end
        if(self.ccw):
            self.rad_len = math.fmod((self.rad_end + 2 * math.pi) - self.rad_start, 2 * math.pi)
        else:
            self.rad_len = math.fmod((self.rad_start + 2 * math.pi) - self.rad_end, 2 * math.pi)
        self.start = self.center + self.radius * np.array([math.cos(self.rad_start), math.sin(self.rad_start)])
        self.end = self.center + self.radius * np.array([math.cos(self.rad_end), math.sin(self.rad_end)])
        self.nb_points = self.nb_points_hint()

    def __str__(self):
        return 'arc: ' + str(self.start) + ' ' + str(self.end) + '\n'

    def length(self):
        return self.radius * self.rad_len

    def generate(self, reverse=False):
        slice = self.nb_points - 1

        if reverse:
            points = self.end
            start_angle = self.rad_end
        else:
            points = self.start
            start_angle = self.rad_start

        if self.ccw != reverse:
            angle_increment = self.rad_len / slice
        else:
            angle_increment = - self.rad_len / slice

        for i in range(1, slice):
            angle = start_angle + i * angle_increment
            points = np.column_stack((points, self.center + self.radius * np.array([math.cos(angle), math.sin(angle)])))

        if reverse:
            return np.column_stack((points, self.start))
        else:
            return np.column_stack((points, self.end))

    def split(self, degree):
        angle_from_start = (self.ccw * 2 - 1) * self.rad_len * degree
        split_angle =  math.fmod(self.rad_start + angle_from_start + 2 * math.pi, 2 * math.pi)
        a = Arc(self.center, self.radius, self.rad_start, split_angle, self.ccw)
        b = Arc(self.center, self.radius, split_angle, self.rad_end, self.ccw)
        return (a, b)

    def reverse(self):
        self.start, self.end = self.end, self.start
        self.rad_start, self.rad_end = self.rad_end, self.rad_start
        self.ccw = not self.ccw

    def nb_points_hint(self):
        max_error = 1e-2
        max_angle = 2 * math.acos(1.0 - max_error / self.radius)
        return math.ceil(self.rad_len / max_angle) + 1

class PathGenerator():
    sync_points = list()

    def __init__(self, items=[]):
        if isinstance(items, list) or isinstance(items, tuple):
            for n in range(1, len(items)):
                items[n].orient_after(items[n-1])
            self.items = list(items)
        elif isinstance(items, PathItem):
            self.items = [items]
        else:
            raise TypeError('items must be a list, a tuple or a PathItem')

    def __str__(self):
        s = "PathGenerator:\n"
        for i in self.items:
            s += str(i)
        return s

    def length(self):
        if len(self.items) == 1:
            return self.items[0].length()
        else:
            return sum([i.length() for i in self.items])

    def item_lengths(self):
        return np.cumsum(np.array([i.length() for i in self.items]))

    def generate(self, reverse = False):
        if len(self.items) == 1:
            return self.items[0].generate(reverse)
        else:
            path = np.array([[],[]])
            ordered_items = reversed(self.items) if reverse else self.items
            for i in ordered_items:
                path = np.column_stack((path[:,:-1], i.generate(reverse)))
            return path

    def split(self, degree, inplace=False):
        degree = degree % 1.0

        cumlen = self.item_lengths()
        splitlen = degree * cumlen[-1]

        if splitlen < epsilon:
            return (PathGenerator(), self)
        if splitlen > cumlen[-1] - epsilon:
            return (self, PathGenerator())

        if len(self.items) == 1:
            a, b = self.items[0].split(degree)
            if inplace:
                self.items = [a, b]
            else:
                return (PathGenerator(a), PathGenerator(b))
        else:
            split_idx = np.argmax(splitlen < cumlen + epsilon)
            if splitlen > cumlen[split_idx] - epsilon:
                if inplace:
                    pass
                else:
                    return (PathGenerator(copy.deepcopy(self.items[:split_idx+1])), PathGenerator(copy.deepcopy(self.items[split_idx+1:])))
            else:
                item_length = self.items[split_idx].length()
                item_degree = (item_length + splitlen - cumlen[split_idx]) / item_length
                a, b = self.items[split_idx].split(item_degree)
                if inplace:
                    self.items.pop(split_idx)
                    self.items.insert(split_idx, b)
                    self.items.insert(split_idx, a)
                else:
                    return (PathGenerator(copy.deepcopy(self.items[:split_idx]) + [a]), PathGenerator([b] + copy.deepcopy(self.items[split_idx+1:])))

    def is_cyclic(self):
        return self.items and self.items[-1].is_followed_by(self.items[0])

    def rotate(self, degree):
        if self.is_cyclic():
            a,b = self.split(degree)
            return b + a
        else:
            # return PathGenerator(copy.deepcopy(self.items))
            return self # TODO not safe, because it's not a copy

    def reverse(self):
        self.sync_points = (1.0 - np.flip(self.sync_points)).tolist()
        self.items.reverse()
        for i in self.items:
            i.reverse()

    def is_followed_by(self, other):
        if not self.items or not other.items:
            return True
        else:
            return self.items[-1].is_followed_by(other.items[0])

    def append(self, item):
        if self.items:
            item.orient_after(self.items[-1])
        self.items.append(item)

    def degrees(self):
        lengths = np.insert(self.item_lengths(), 0, 0.0)
        return lengths / lengths[-1]

    def synchronize(a, b):
        if not a.items or not b.items:
            return a, b # TODO not safe, because it's not a copy

        tmp = a.sync_points
        a = PathGenerator(copy.deepcopy(a.items))
        a.sync_points = tmp
        tmp = b.sync_points
        b = PathGenerator(copy.deepcopy(b.items))
        b.sync_points = tmp

        #cut a and b on sync points if no near point available
        nb_sync_points = min(len(a.sync_points), len(b.sync_points))
        for i in range(nb_sync_points):
            a.split(a.sync_points[i], True)
            b.split(b.sync_points[i], True)

        #prepare list of sync interval for a
        tmp = a.sync_points[:nb_sync_points]
        a_sync_intervals = [list(x) for x in zip([0.0] + tmp, tmp + [1.0])]

        #prepare list of sync interval for b
        tmp = b.sync_points[:nb_sync_points]
        b_sync_intervals = [list(x) for x in zip([0.0] + tmp, tmp + [1.0])]

        #iterate through both interval lists
        all_cut = []
        for a_itv, b_itv in zip(a_sync_intervals, b_sync_intervals):
            # take all a degree between length a_itv[0]+epsilon et a_itv[1]-epsilon
            a_lengths = a.item_lengths()[:-1]
            a_mask = (a_lengths >= a_itv[0] * a.length() + epsilon) & (a_lengths <= a_itv[1] * a.length() - epsilon)
            a_deg = a.degrees()[1:-1][a_mask]

            # take all b degree between length b_itv[0]+epsilon et b_itv[1]-epsilon
            b_lengths = b.item_lengths()[:-1]
            b_mask = (b_lengths >= b_itv[0] * b.length() + epsilon) & (b_lengths <= b_itv[1] * b.length() - epsilon)
            b_deg = b.degrees()[1:-1][b_mask]

            filtered = []
            all_deg = np.sort(np.concatenate([a_deg, b_deg]))
            while len(all_deg) >= 2:
                if (all_deg[0] * a.length() <= all_deg[1] * a.length() - epsilon
                and all_deg[0] * b.length() <= all_deg[1] * b.length() - epsilon):
                    filtered.append(all_deg[0])
                    all_deg = all_deg[1:]
                else:
                    all_deg = all_deg[2:]
            if all_deg:
                filtered.append(all_deg[0])

            all_cut += filtered

        for i in all_cut:
            a.split(i, True)
            b.split(i, True)

        for i in range(len(a.items)):
            nb_points = max(a.items[i].nb_points_hint(), b.items[i].nb_points_hint())
            a.items[i].set_nb_points(nb_points)
            b.items[i].set_nb_points(nb_points)

        return (a, b)

    def __add__(self, other):
        if(not self.is_followed_by(other)):
            raise ValueError('Tried to link unlinkable paths')
        return PathGenerator(self.items + other.items)
