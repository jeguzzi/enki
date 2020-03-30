import pyenki
import math

# Create a world and place two thymios in front of themselves
world = pyenki.World()
thymio1 = pyenki.Thymio2()
thymio1.position = (0, 0)
world.add_object(thymio1)
thymio2 = pyenki.Thymio2()
thymio2.position = (30, 0)
thymio2.angle = math.pi
world.add_object(thymio2)

# Enable the communication and let them send different payloads
thymio1.prox_comm_enable = True
thymio2.prox_comm_enable = True
thymio1.prox_comm_tx = 111
thymio2.prox_comm_tx = 222

for _ in range(100):
    world.step(0.1)
    # Check which messages they are receiving
    print('thymio1: ', '\t\n'.join(f'Got message {e.rx} [{e.payloads} {e.intensities}]'
                                   for e in thymio1.prox_comm_events))
    print('thymio2: ', '\t\n'.join(f'Got message {e.rx} [{e.payloads} {e.intensities}]'
                                   for e in thymio2.prox_comm_events))
