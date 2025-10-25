import pyenki
import random


class MyEPuck(pyenki.EPuck):

    def __init__(self):
        super(MyEPuck, self).__init__(proximity=False)
        self.timeout = 10

    def controlStep(self, dt):
        if self.timeout == 0:
            self.left_wheel_target_speed = random.uniform(-100, 100)
            self.right_wheel_target_speed = random.uniform(-100, 100)
            self.timeout = random.randint(1, 10)
        else:
            self.timeout -= 1
        # print(id(self), self.position)


w = pyenki.World()

for i in range(0, 10):
    for j in range(0, 10):
        e = MyEPuck()
        e.position = (i * 10, j * 10)
        w.add_object(e)

w.run_in_viewer()
