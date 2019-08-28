from abc import ABC, abstractmethod
import numpy as np
import math, copy

epsilon = 1e-3

class PathItem(ABC):
    @abstractmethod
    def length(self):
        pass

    @abstractmethod
    def generate(self):
        pass

    @abstractmethod
    def get_point(self, degree):
        pass

    @abstractmethod
    def split(self, degrees):
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

class Line(PathItem):
    def __init__(self, start, end):
        self.start = np.array(start)
        self.end = np.array(end)
        self.nb_points = self.nb_points_hint()

    def __str__(self):
        return 'line: ' + str(self.start) + ' ' + str(self.end) + '\n'

    def length(self):
        return np.linalg.norm(self.start - self.end)

    def generate(self):
        return np.linspace(self.start, self.end, num=self.nb_points, axis=1)

    def get_point(self, degree):
        return self.start + (self.end - self.start) * degree

    def split(self, degrees):
        prev_split = self.start
        slices = []
        for i in degrees:
            split = self.get_point(i)
            slices.append(Line(prev_split, split))
            prev_split = split
        slices.append(Line(prev_split, self.end))
        return slices

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
            self.rad_len = math.fmod((self.rad_end - self.rad_start) + 2 * math.pi, 2 * math.pi)
        else:
            self.rad_len = math.fmod((self.rad_start - self.rad_end) + 2 * math.pi, 2 * math.pi)
        self.start = self.center + self.radius * np.array([math.cos(self.rad_start), math.sin(self.rad_start)])
        self.end = self.center + self.radius * np.array([math.cos(self.rad_end), math.sin(self.rad_end)])
        self.nb_points = self.nb_points_hint()

    def __str__(self):
        return 'arc: ' + str(self.start) + ' ' + str(self.end) + '\n'

    def length(self):
        return self.radius * self.rad_len

    def generate(self):
        slice = self.nb_points - 1

        points = self.start
        start_angle = self.rad_start

        if self.ccw:
            angle_increment = self.rad_len / slice
        else:
            angle_increment = - self.rad_len / slice

        for i in range(1, slice):
            angle = start_angle + i * angle_increment
            points = np.column_stack((points, self.center + self.radius * np.array([math.cos(angle), math.sin(angle)])))

        return np.column_stack((points, self.end))

    def get_point(self, degree):
        angle_from_start = (self.ccw * 2 - 1) * self.rad_len * degree
        angle = math.fmod(self.rad_start + angle_from_start + 2 * math.pi, 2 * math.pi)
        return self.center + self.radius * np.array([math.cos(angle), math.sin(angle)])

    def split(self, degrees):
        prev_split = self.rad_start
        slices = []
        for i in degrees:
            angle_from_start = (self.ccw * 2 - 1) * self.rad_len * i
            split = math.fmod(self.rad_start + angle_from_start + 2 * math.pi, 2 * math.pi)
            slices.append(Arc(self.center, self.radius, prev_split, split, self.ccw))
            prev_split = split
        slices.append(Arc(self.center, self.radius, prev_split, self.rad_end, self.ccw))
        return slices

    def reverse(self):
        self.start, self.end = self.end, self.start
        self.rad_start, self.rad_end = self.rad_end, self.rad_start
        self.ccw = not self.ccw

    def nb_points_hint(self):
        max_error = 1e-2
        max_angle = 2 * math.acos(1.0 - max_error / self.radius)
        return max(2, math.ceil(self.rad_len / max_angle) + 1)

class PathGenerator():
    def __init__(self, items=[]):
        self.sync_points = list()
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
        return np.array([i.length() for i in self.items])

    def cumulated_lengths(self):
        return np.cumsum(self.item_lengths())

    def generate(self):
        if len(self.items) == 1:
            return self.items[0].generate()
        else:
            path = np.array([[],[]])
            for i in self.items:
                path = np.column_stack((path[:,:-1], i.generate()))
            return path

    def slice(self, degrees):
        if degrees.size == 0:
            return

        cumlen = self.cumulated_lengths()
        deglen = degrees * cumlen[-1]

        item_idx = []
        current_item = 0
        for i in deglen:
            while i > cumlen[current_item]:
                current_item += 1
            item_idx.append(current_item)

        item_deglen = [deglen[i] - (np.insert(cumlen, 0, 0.0))[item_idx[i]] for i in range(len(deglen))]

        res = []
        item_deg = []
        prev_item = -1
        itemlen = self.item_lengths()
        for i in range(len(item_idx)):
            if item_idx[i] > prev_item + 1:
                res += self.items[prev_item+1 : item_idx[i]]
            prev_item = item_idx[i]
            item_deg.append(item_deglen[i] / itemlen[item_idx[i]])
            if i+1 > len(item_idx)-1 or item_idx[i+1] > item_idx[i]:
                res += self.items[item_idx[i]].split(item_deg)
                item_deg = []
        res += self.items[item_idx[-1]+1:]
        self.items = res

    def decompose_degree(self, degree):
        degree = degree % 1.0
        if len(self.items) == 1:
            return (0, degree)
        else:
            cumlen = self.cumulated_lengths()
            splitlen = degree * cumlen[-1]
            split_idx = np.argmax(cumlen > splitlen)
            item_length = self.items[split_idx].length()
            item_degree = (item_length + splitlen - cumlen[split_idx]) / item_length
            return (split_idx, item_degree)

    def get_point(self, degree):
        idx, deg = self.decompose_degree(degree)
        return self.items[idx].get_point(deg)

    def split(self, degree):
        split_idx, item_degree = self.decompose_degree(degree)
        a, b = self.items[split_idx].split([item_degree])
        return (PathGenerator(copy.deepcopy(self.items[:split_idx]) + [a]), PathGenerator([b] + copy.deepcopy(self.items[split_idx+1:])))

    def is_cyclic(self):
        return self.items and self.items[-1].is_followed_by(self.items[0])

    def rotate(self, degree):
        if self.is_cyclic():
            len = self.length()
            splitlen = degree * len
            if splitlen < epsilon or splitlen > len - epsilon:
                return self
            a,b = self.split(degree)
            gen = b + a
            gen.sync_points = np.mod(np.array(self.sync_points) - degree, 1).tolist()
            gen.sync_points.sort()
            return gen
        else:
            return self.copy()

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
        cumlen = self.cumulated_lengths()
        if cumlen.size > 0:
            return np.insert(cumlen, 0, 0.0) / cumlen[-1]
        else:
            return np.array([])

    def synchronize(a, b):
        a = a.copy()
        b = b.copy()
        if a.items and b.items:
            prev = (0.0, 0.0)
            n = min(len(a.sync_points), len(b.sync_points))
            all_b_cut = np.array([])
            all_a_cut = np.array([])
            for i in zip(a.sync_points[:n] + [2.0], b.sync_points[:n] + [2.0]):
                len_a = min(1.0, i[0]) - prev[0]
                len_b = min(1.0, i[1]) - prev[1]

                a_deg = a.degrees()[1:-1]
                a_deg = a_deg[(a_deg >= prev[0]) & (a_deg < i[0])]
                a_deg = ((a_deg - prev[0]) * (len_b / len_a)) + prev[1]
                all_b_cut = np.concatenate((all_b_cut, a_deg))

                b_deg = b.degrees()[1:-1]
                b_deg = b_deg[(b_deg >= prev[1]) & (b_deg < i[1])]
                b_deg = ((b_deg - prev[1]) * (len_a / len_b)) + prev[0]
                all_a_cut = np.concatenate((all_a_cut, b_deg))

                prev = i

            all_a_cut = np.concatenate((all_a_cut, a.sync_points[:n]))
            all_b_cut = np.concatenate((all_b_cut, b.sync_points[:n]))
            a.slice(np.sort(all_a_cut))
            b.slice(np.sort(all_b_cut))

            if len(a.items) != len(b.items):
                raise Error('Path synchronisation failure')
            for i in range(len(a.items)):
                nb_points = max(a.items[i].nb_points_hint(), b.items[i].nb_points_hint())
                a.items[i].set_nb_points(nb_points)
                b.items[i].set_nb_points(nb_points)
        return a, b

    def close_to(self, p):
        #NOTE if better performance required, this three variables could be stored between calls
        generated = self.generate()
        degrees_buf = self.degrees()
        length_buf = self.length()
        idx_table = [(n, i.nb_points-1) for n, i in enumerate(self.items)]

        if generated.size == 0:
            return None, None

        a = generated[:,:-1]
        b = generated[:,1:]
        c = np.array(p).reshape((2,1))

        ab = b - a
        ac = c - a

        with np.errstate(divide='ignore', invalid='ignore'):
            nearest_in_seg = np.nan_to_num(np.clip(np.sum(ab*ac, axis=0) / np.sum(np.square(ab), axis=0), 0.0, 1.0), 0.0)
        nearest_in_abs = nearest_in_seg * ab + a
        dist_to_seg = np.linalg.norm(nearest_in_abs - c, axis=0)
        id = np.argmin(dist_to_seg)

        idx = id
        deg = 0
        for i in idx_table:
            if idx >= i[1]:
                idx -= i[1]
            else:
                deg = degrees_buf[i[0]] + (idx+nearest_in_seg[id]) * self.items[i[0]].length() / (length_buf * i[1])
                break

        p_dict = {'pos':nearest_in_abs[:, id], 'dist':dist_to_seg[id], 'deg':deg}

        if not self.sync_points:
            return p_dict, None

        sync_pos = self.sync_points_pos()
        sync_dist = np.linalg.norm(sync_pos - c, axis=0)
        id = np.argmin(sync_dist)
        s_dict = {'pos':sync_pos[:, id], 'dist':sync_dist[id], 'deg':self.sync_points[id]}

        return p_dict, s_dict

    def sync_points_pos(self):
        sync_pos = np.array([[], []])
        for i in self.sync_points:
            sync_pos = np.column_stack((sync_pos, self.get_point(i)))
        return sync_pos

    def remove_sync_point(self, deg):
        self.sync_points.remove(deg)

    def add_sync_point(self, deg):
        self.sync_points.append(deg)
        self.sync_points.sort()

    def copy(self):
        cp = PathGenerator(copy.deepcopy(self.items))
        cp.sync_points = self.sync_points
        return cp

    def __add__(self, other):
        if(not self.is_followed_by(other)):
            raise ValueError('Tried to link unlinkable paths')
        return PathGenerator(self.items + other.items)
