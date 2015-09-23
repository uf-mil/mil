import numpy as np
from vispy import gloo
from vispy import app
from vispy.util.transforms import perspective, translate, rotate
from sub8_sim_tools.rendering import Sphere, Box, World


class Canvas(app.Canvas):
    def __init__(self):
        app.Canvas.__init__(self, keys='interactive', size=(800, 800))
        self.size = (800, 800)

        self.translate = np.array([0.0, 0.0, 0.0])
        self.rotate = np.array([0.0, 0.0, 0.0])

        gloo.set_state(depth_test=True, blend=True)
        self._timer = app.Timer('auto', connect=self.on_timer, start=True)
        self.physics_timer = app.Timer(1 / 29., connect=self.step_physics, start=True)
        self.clock = 0
        self.view = np.eye(4)
        self.show()

    def on_timer(self, event):
        self.clock += 0.1
        self.update()

    def on_resize(self, event):
        width, height = event.size
        gloo.set_viewport(0, 0, width, height)
    
    def step_physics(self, event):
        pass

    def on_draw(self, event):
        gloo.set_viewport(0, 0, *self.size)
        gloo.clear(color=True, depth=True)
        self.world.draw(self.view)

    def on_key_press(self, event):
        self.translate = np.array([0.0, 0.0, 0.0])
        self.rotate = np.array([0.0, 0.0, 0.0])
        if(event.text.lower() == 'p'):
            print(repr(self.view))
        elif(event.text.lower() == 'd'):
            self.translate[0] += -0.3
        elif(event.text.lower() == 'a'):
            self.translate[0] += 0.3
        elif(event.text.lower() == ' '):
            self.translate[1] += -0.3
        elif(event.text.lower() == 'c'):
            self.translate[1] += 0.3
        elif(event.text.lower() == 's'):
            self.translate[2] += -0.3
        elif(event.text.lower() == 'w'):
            self.translate[2] += 0.3

        elif(event.text == 'x'):
            self.rotate += [2, 0, 0]
        elif(event.text == 'X'):
            self.rotate += [-2, 0, 0]
        elif(event.text == 'y'):
            self.rotate += [0, 2, 0]
        elif(event.text == 'Y'):
            self.rotate += [0, -2, 0]
        elif(event.text == 'z'):
            self.rotate += [0, 0, 2]
        elif(event.text == 'Z'):
            self.rotate += [0, 0, -2]

        self.view = self.view.dot(
            rotate(self.rotate[0], (1, 0, 0))).dot(
            rotate(self.rotate[1], (0, 1, 0))).dot(
            rotate(self.rotate[2], (0, 0, 1))
        )
        self.view = self.view.dot(translate(list(self.translate)))


if __name__ == '__main__':
    c = Canvas()
    c.world = World()
    c.world.add_sphere((0.0, 0.0, 0.0), 1.0, (0.0, 50.0, 0.0))
    c.world.add_sphere((0.0, 0.0, 0.3), 1.0, (100., 50.0, 0.0))
    c.world.add_sphere((0.0, 0.8, 0.0), 0.4, (100., 50.0, 0.0))
    c.world.add_sphere((0.8, 0.0, 0.5), 1.0, (100., 50.0, 0.0))
    c.world.add_sphere((0.0, -0.8, -8), 0.4, (100., 50.0, 0.0))
    c.world.add_box((0.8, 0.8, 5), 5.0, 2.0, 3.0, (50, 100, 100))
    c.world.add_plane((0.0, 0.0, -1.0), 20.0, 20.0)
    c.world.add_point_light((0.0, 0.0, 0.0), (0.1, 0.8, 0.5))

    app.run()