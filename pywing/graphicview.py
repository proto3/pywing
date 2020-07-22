from PyQt5 import QtCore
import numpy as np
from vispy import scene, gloo
from cuttingpathvisual import CuttingPathVisual
import triangle

gloo.gl.use_gl('glplus')

def triangulate(path):
    if np.size(path, 1) > 2:
        dup_idx = np.argwhere(np.all(np.isclose(path[:,1:], path[:,:-1], atol=1e-3), axis=0)).flatten()
        if np.allclose(path[:,0], path[:,-1], atol=1e-3):
            dup_idx = np.append(dup_idx, 0)
        path = np.delete(path, dup_idx, axis=1).transpose()

        n = np.size(path, 0)
        segments = np.column_stack((np.arange(n), np.arange(1, n+1)))
        segments[-1][1] = 0

        result = triangle.triangulate({'vertices': path, 'segments': segments}, "p")
        return result['vertices'], result['triangles']

def on_mouse_press(event):
    pass
    # print("press",event.pos, event.button, event.delta)

def on_mouse_move(event):
    pass
    # print("move",event.pos, event.button, event.delta)

def lines(n, normal):
    alternate = np.empty((2*n,))
    alternate[::2] = 0
    alternate[1::2] = 1
    if normal == 'x':
        return np.vstack((np.zeros(2*n), np.repeat(np.arange(0, n, 1), 2), alternate))
    if normal == 'y':
        return np.vstack((np.repeat(np.arange(0, n, 1), 2), np.zeros(2*n), alternate))
    if normal == 'z':
        return np.vstack((np.repeat(np.arange(0, n, 1), 2), alternate, np.zeros(2*n)))

def machine_grid(x, y, z, step):
    x_steps = int(x/step)+1
    y_steps = int(y/step)+1
    z_steps = int(z/step)+1

    xlines = lines(x_steps, 'z')
    ylines = lines(y_steps, 'z')
    ylines = np.take(ylines, [1,0,2], axis=0)
    gridxy = np.column_stack((xlines * np.array([[step], [y], [0]]),
                              ylines * np.array([[x], [step], [0]])))

    xlines = lines(x_steps, 'y')
    zlines = lines(z_steps, 'y')
    zlines = np.take(zlines, [2,1,0], axis=0)
    gridxz = np.column_stack((xlines * np.array([[step], [0], [z]]),
                              zlines * np.array([[x], [0], [step]])))

    return np.column_stack((gridxy, gridxz, gridxz + np.array([[0],[y],[0]]))).transpose()

class GraphicView(QtCore.QObject):
    def __init__(self, cut_processor, machine):
        super().__init__()
        self._cut_proc = cut_processor
        self._machine = machine
        length, width, height = self._machine.get_dimensions()

        self.canvas = scene.SceneCanvas(keys='interactive', size=(800, 600), create_native=True)
        self.camera = scene.cameras.TurntableCamera(fov=45.0, elevation=30.0, azimuth=30.0, roll=0.0, distance=None)
        self.view = self.canvas.central_widget.add_view(self.camera)

        # set ortho
        self.camera.fov = 0.0

        self.plot_l = scene.LinePlot(width=2.0, color=(0.91, 0.31, 0.22, 1.0), parent=self.view.scene)
        self.plot_r = scene.LinePlot(width=2.0, color=(0.18, 0.53, 0.67, 1.0), parent=self.view.scene)
        self.mplot_l = scene.LinePlot(width=2.0, color=(1.0, 0.0, 0.0, 1.0), parent=self.view.scene)
        self.mplot_r = scene.LinePlot(width=2.0, color=(1.0, 0.0, 0.0, 1.0), parent=self.view.scene)
        self.face_l = scene.visuals.Mesh(color=(0.82, 0.28, 0.20, 1.0), mode='triangles', parent=self.view.scene)
        self.face_r = scene.visuals.Mesh(color=(0.16, 0.48, 0.60, 1.0), mode='triangles', parent=self.view.scene)

        CuttingPathNode = scene.visuals.create_visual_node(CuttingPathVisual)
        self.cutting_path = CuttingPathNode(color=(0.5, 0.5, 0.5, 1), parent=self.view.scene)

        m_grid = machine_grid(length, width, height, 50)
        self.mgrid_visual = scene.visuals.Line(pos=m_grid, color=(0.8,0.8,0.8,0.5), connect='segments', antialias=True, parent=self.view.scene)

        self.canvas.events.mouse_press.connect(on_mouse_press)
        self.canvas.events.mouse_move.connect(on_mouse_move)

        self._cut_proc.update.connect(self.draw)

    def draw(self):
        path_l, path_r = self._cut_proc.get_paths()
        self.plot_l.set_data(path_l.transpose(), symbol=None)
        self.plot_r.set_data(path_r.transpose(), symbol=None)

        assert(not np.any(np.isnan(path_l)))
        assert(not np.any(np.isnan(path_r)))

        if path_l.size > 0:
            v, f = triangulate(path_l[::2,1:-1]) # remove lead trajectories and Y axis to triangulate
            v = np.insert(v, 1, path_l[1][0], axis=1) # re-add Y axis
            self.face_l.set_data(vertices=v, faces=f)

        if path_r.size > 0:
            v, f = triangulate(path_r[::2,1:-1]) # remove lead trajectories and Y axis to triangulate
            v = np.insert(v, 1, path_r[1][0], axis=1) # re-add Y axis
            self.face_r.set_data(vertices=v, faces=f)

        if path_l.size > 0 and path_r.size > 0:
            mpath_l, mpath_r = self._cut_proc.get_machine_paths()
            self.mplot_l.set_data(mpath_l.transpose(), symbol=None)
            self.mplot_r.set_data(mpath_r.transpose(), symbol=None)

            vertices = np.hstack((path_l, path_r)).transpose()

            f = np.repeat(np.arange(0, np.size(path_l, 1)), 2)[1:-1]
            fc = f.copy() + np.size(path_l, 1)
            faces = np.empty(f.size * 2, dtype=np.int32)
            faces[0::2] = f
            faces[1::2] = fc
            faces = np.reshape(faces, (int(faces.size/4), 4))

            self.cutting_path.set_data(vertices=vertices, faces=faces)
            # self.camera.set_range()
