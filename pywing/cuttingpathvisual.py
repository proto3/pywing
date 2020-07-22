import numpy as np
from vispy import visuals, gloo, geometry, color as clr, ext

shading_vertex_template = """
varying out vec3 v_position;

void main() {
    gl_Position = $transform(vec4($position, 1.0));
    v_position = $position;
}
"""

shading_geometry_template = """
#version 150 core
in vec3 v_position[];
out vec3 g_normal_vec;

layout(lines_adjacency) in;
layout(triangle_strip, max_vertices = 52) out;

void main(void) {
    int n = 25;

    vec3 v_normal_vec[4];
    v_normal_vec[0] = normalize(cross(v_position[1]-v_position[0], v_position[2]-v_position[0]));
    v_normal_vec[1] = normalize(cross(v_position[3]-v_position[1], v_position[0]-v_position[1]));
    v_normal_vec[2] = normalize(cross(v_position[0]-v_position[2], v_position[3]-v_position[2]));
    v_normal_vec[3] = normalize(cross(v_position[2]-v_position[3], v_position[1]-v_position[3]));

    g_normal_vec = v_normal_vec[2];
    gl_Position = gl_in[2].gl_Position;
    EmitVertex();

    g_normal_vec = v_normal_vec[0];
    gl_Position = gl_in[0].gl_Position;
    EmitVertex();

    for(int i=1;i<n-1;++i)
    {
        float a = float(n-i)/(n-1);
        float b = float(i)/(n-1);

        g_normal_vec = normalize(v_normal_vec[2] * a + v_normal_vec[3] * b);
        gl_Position = gl_in[2].gl_Position + (gl_in[3].gl_Position - gl_in[2].gl_Position) * b;
        EmitVertex();

        g_normal_vec = normalize(v_normal_vec[0] * a + v_normal_vec[1] * b);
        gl_Position = gl_in[0].gl_Position + (gl_in[1].gl_Position - gl_in[0].gl_Position) * b;
        EmitVertex();
    }

    g_normal_vec = v_normal_vec[3];
    gl_Position = gl_in[3].gl_Position;
    EmitVertex();

    g_normal_vec = v_normal_vec[1];
    gl_Position = gl_in[1].gl_Position;
    EmitVertex();

    EndPrimitive();
}
"""

shading_fragment_template = """
varying vec3 g_normal_vec;
void main() {
    vec3 lights[4];
    lights[0] = vec3(1.0, 0.0, 1.0);
    lights[1] = vec3(1.0, 0.0, -1.0);
    lights[2] = vec3(-1.0, 0.0, 1.0);
    lights[3] = vec3(-1.0, 0.0, -1.0);

    vec4 light_colors[4];
    light_colors[0] = vec4(1.0, 0.0, 0.0, 1.0);
    light_colors[1] = vec4(0.0, 1.0, 0.0, 1.0);
    light_colors[2] = vec4(0.0, 0.0, 1.0, 1.0);
    light_colors[3] = vec4(1.0, 0.0, 0.0, 1.0);

    vec4 diffuse_color = vec4(0,0,0,0);
    for(int i=0;i<4;i++)
    {
        //DIFFUSE
        float diffusek = dot(lights[i], g_normal_vec);
        // clamp, because 0 < theta < pi/2
        diffusek  = clamp(diffusek, 0.0, 1.0);
        diffuse_color += light_colors[i] * diffusek;
    }

    gl_FragColor = $base_color * ($ambientk + diffuse_color);
}
"""

class CuttingPathVisual(visuals.Visual):
    def __init__(self, vertices=None, faces=None, color=(0.5, 0.5, 1, 1), **kwargs):
        visuals.Visual.__init__(self, vcode=shading_vertex_template,
                                      fcode=shading_fragment_template,
                                      gcode=shading_geometry_template,
                                      **kwargs)

        self.set_gl_state('translucent', depth_test=True, cull_face=False)
        self.shading = 'smooth'
        self._draw_mode = 'lines_adjacency'

        # Define buffers
        self.vv = None
        self._vertices = gloo.VertexBuffer(np.zeros((0, 3), dtype=np.float32))
        self._faces = gloo.IndexBuffer()
        self._color = clr.Color(color)

        # Init
        self._bounds = None

        # Note we do not call subclass set_data -- often the signatures
        # do no match.
        CuttingPathVisual.set_data(self, vertices=vertices, faces=faces)

        self.freeze()


    def set_data(self, vertices=None, faces=None):
        if vertices is not None:
            self.vv = np.ascontiguousarray(vertices)
        else:
            self.vv = None

        self.ff = faces

        meshdata = geometry.MeshData(vertices=vertices, faces=faces)
        self._bounds = meshdata.get_bounds()

        self._data_changed = True
        self.update()

    def _update_data(self):

        if self.vv is None:
            return False

        self._vertices.set_data(self.vv, convert=True)
        self._faces.set_data(self.ff, convert=True)
        self._index_buffer = self._faces

        self.shared_program.vert['position'] = self._vertices
        self.shared_program.frag['base_color'] = self._color.rgba
        self.shared_program.frag['ambientk'] = [0.3, 0.3, 0.3, 1.0]
        self._data_changed = False

    def _prepare_draw(self, view):
        if self._data_changed:
            if self._update_data() is False:
                return False
            self._data_changed = False

    def draw(self, *args, **kwds):
        visuals.Visual.draw(self, *args, **kwds)

    @staticmethod
    def _prepare_transforms(view):
        tr = view.transforms.get_transform()
        view.view_program.vert['transform'] = tr

    def _compute_bounds(self, axis, view):
        if self._bounds is None:
            return None
        return self._bounds[axis]
