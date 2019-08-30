import numpy as np
import math

class Path():
    def __init__(self):
        super().__init__()
        self.initial_path = np.array([[],[]])
        self.final_path   = np.array([[],[]])
        self.s = 1.0        # Scale
        self.r = 0.0        # Rotation
        self.t = [0.0, 0.0] # Translation
        self.k = 0.0        # Kerf width
        self.l = 10.0       # Lead in/out length

        self.tr_mat   = np.array([])
        self.lead_in  = np.array([])
        self.lead_out = np.array([])

    def __str__(self):
        return str(self.final_path)

    def export_tuple(self):
        return (self.s, self.k, self.initial_path)

    def import_tuple(self, tuple):
        self.s, self.k, self.initial_path = tuple
        self._apply_transform()
        # self.reset.emit()

    def get_path(self):
        return self.final_path

    def get_boundaries(self):
        # return [xmin, ymin, xmax, ymax]
        return np.concatenate((np.amin(self.final_path, axis=1), np.amax(self.final_path, axis=1)))

    def scale(self, s):
        self.s = s
        self._apply_transform()

    def rotate(self, r):
        self.r = r
        self._apply_transform()

    def translate_x(self, t):
        self.t[0] = t
        self._apply_transform()

    def translate_y(self, t):
        self.t[1] = t
        self._apply_transform()

    def set_kerf_width(self, k):
        self.k = k
        self._apply_transform()

    def set_lead_size(self, l):
        self.l = l
        self._apply_transform()

    def _apply_transform(self):
        if self.initial_path.size == 0:
            return

        # apply kerf width correction
        self._apply_kerf()

        r_rad = self.r / 180 * math.pi
        a = self.s * math.cos(r_rad)
        b = self.s * math.sin(r_rad)

        #prepare transform matrix
        self.tr_mat = np.array([[a,-b, self.t[0]],
                                [b, a, self.t[1]],
                                [0, 0,         1]])

        # apply matrix
        self.final_path = np.dot(self.tr_mat, np.insert(self.kerf_path, 2, 1.0, axis=0))[:-1]

        # append lead in and out to path
        i = 1
        while True:
            if i == np.size(self.final_path, 1):
                raise Error('Path to short to compute lead direction')
            a = self.final_path[:, i]
            b = self.final_path[:, 0]
            length = np.linalg.norm(b-a)
            if length > 1e-3:
                self.lead_in = b + (b-a) * self.l / length
                break
            else:
                i+=1

        self.lead_in = self._compute_lead(self.final_path)
        self.lead_out = self._compute_lead(np.flip(self.final_path, axis=1))
        self.final_path = np.column_stack((self.lead_in, self.final_path, self.lead_out))

    def _apply_kerf(self):
        ini = self.initial_path
        dup_idx = np.argwhere(np.all(np.equal(ini[:,1:], ini[:,:-1]), axis=0)).flatten()
        select = np.delete(ini, dup_idx, axis=1)
        delta = select[:,1:] - select[:,:-1]
        extended = np.column_stack((delta[:,0], delta, delta[:,-1]))
        angle = np.arctan2(extended[1], extended[0])
        mid_angle = angle[:-1] + np.mod(angle[1:] - angle[:-1] + math.pi, 2*math.pi) / 2
        radius = self.k / self.s
        offset = np.stack((np.cos(mid_angle), np.sin(mid_angle))) * radius
        select += offset
        for i in dup_idx:
            select = np.insert(select, i, select[:,i-1], axis=1)
        self.kerf_path = select

    def _compute_lead(self, path):
        i = 1
        while i < np.size(path, 1):
            a = path[:, i]
            b = path[:, 0]
            length = np.linalg.norm(b-a)
            if length > 1e-3:
                return b + (b-a) * self.l / length
            else:
                i+=1
        raise Error('Path to short to compute lead direction')
