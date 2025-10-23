import pyenki
import math


class MyEPuck(pyenki.EPuck):

    def __init__(self):
        super().__init__(proximity=True, camera=True)

    def controlStep(self, dt):
        self.left_wheel_target_speed = 0.1
        self.right_wheel_target_speed = 0.2
        print('Control step')
        print(f'pos: {self.position}')
        print(f'IR dists: {self.prox_distances}')
        assert (not any(map(math.isnan, self.prox_distances)))
        print(f'IR values: {self.prox_values}')
        assert (not any(map(math.isnan, self.prox_values)))
        print(f'Cam image: {self.camera_image}')


w = pyenki.World()
e = MyEPuck()
#e = pyenki.EPuck()
w.add_object(e)

for i in range(10):
    w.step(0.05)
    print('')
