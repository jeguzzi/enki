from __future__ import annotations

import math
import warnings
from typing import TYPE_CHECKING, SupportsFloat

from ... import Array1D, Controller, PhysicalObject, Thymio2

if TYPE_CHECKING:
    from typing import ParamSpec

    from thymio_behaviors import Behavior, Callback

    P = ParamSpec('P')


def smod(value: int, n: int) -> int:
    return int((value + 2**(n - 1)) % 2**n - 2**(n - 1))


def umod(value: int, n: int) -> int:
    return int(value % 2**n)


def to_list(values: Array1D) -> list[int]:
    return [round(x) for x in values]


def to_space_i(value: float) -> int:
    return smod(math.floor(value / 16.6 * 500), 16)


def from_space_i(value: int) -> float:
    return 16.6 * value / 500


def to_led_i(value: float) -> int:
    return math.floor(max(0, min(1, value)) * 32)


def from_led_i(value: int) -> float:
    return value / 32


def to_acc_i(value: float) -> int:
    return math.floor(value / 981 * 23)


class Thymio2AsebaAdapter:
    """
    Exposes an interface that mimics the
    `Aseba interface of the Thymio <http://aseba.wikidot.com/en:thymioapi>`_,
    where

    - Aseba variables ``x.y.z`` are exposed as Python properties ``x_y_z``
    - Aseba functions ``x.y.z`` are exposed as Python methods ``call_x_y_z``
      with the same number of integer arguments.

    .. note::

       This adapter does not implement Aseba events,
       which is the primary interface of Aseba controllers.
       Instead, it allows to write procedural controllers,
       which, although structurally different, can be functionally equivalent.

    For example, we can mimic the following Aseba piece of code ::

        call leds.top(32, 0, 0)
        if prox.horizontal[2] > 2000 then
            motor.left.target = 123
        else
            motor.left.target = 0
        end


    with ::

        # Create a Thymio
        thymio = pyenki.Thymio2()
        # Wrap it in the Aseba-like interface
        aseba = Thymio2AsebaAdapter(thymio)

        aseba.update()

        # Use a similar API
        aseba.call_leds_top(32, 0, 0)
        if aseba.prox_horizontal[2] > 2000:
            aseba.motor_left_target = 123
        else
            aseba.motor_left_target = 0
        end

        aseba.actuate()

    Like Aseba, it keeps the most-recent received
    proximity communication message in ``prox.comm.rx._payloads``,
    ``prox.comm.rx._intensities``, and ``prox.comm.rx``.
    In addition, it records in :py:attr:`prox_comm_buffer`
    the **list** of messages received in the last update step.
    """

    def __init__(self, thymio: Thymio2) -> None:
        """
        Constructs a new instance.

        :param      thymio:  The robot
        """
        self.thymio = thymio
        self._prox_comm_buffer: list[tuple[int, list[int]]] = []

        # ASEBA VARIABLES
        self._fwversion = [14, 0]
        self._id = 0
        self._imot = [0, 0]
        self._integrator = [0, 0]
        self._productId = 8
        self._vbat = [0, 0]
        self.acc = [0, 0, 0]
        self.acc__tap = 0
        self.button_backward = 0
        self.button_left = 0
        self.button_center = 0
        self.button_forward = 0
        self.button_right = 0
        self.buttons__mean = [0] * 5
        self.buttons__noise = [0] * 5
        self.buttons__raw = [0] * 5
        self.event_args = [0] * 32
        self.event_source = 0
        self.temperature = 0
        self.leds_bottom_left = [0] * 3
        self.leds_bottom_right = [0] * 3
        self.leds_circle = [0] * 8
        self.leds_top = [0] * 3
        self.mic__mean = 0
        self.mic_intensity = 0
        self.mic_threshold = 0
        self.motor_left_pwm = 0
        self.motor_left_speed = 0
        self.motor_left_target = 0
        self.motor_right_pwm = 0
        self.motor_right_speed = 0
        self.motor_right_target = 0
        self.prox_comm_rx = 0
        self.prox_comm_rx__intensities = [0] * 7
        self.prox_comm_rx__payloads = [0] * 7
        self.prox_comm_tx = 0
        self.prox_ground_ambiant = [0, 0]
        self.prox_ground_delta = [0, 0]
        self.prox_ground_reflected = [0, 0]
        self.prox_horizontal = [0] * 7
        self.rc5_address = 0
        self.rc5_command = 0
        self.sd_present = 0
        self.timer_period = [0, 0]
        self.temperature = 0

    @property
    def prox_comm_buffer(self) -> list[tuple[int, list[int]]]:
        return self._prox_comm_buffer

    def actuate(self) -> None:
        """
        Forwards the commands to the robots.
        Should be called after setting Aseba variables
        in a control step.
        """
        self.thymio.prox_comm_tx = smod(self.prox_comm_tx, 16)
        self.thymio.left_wheel_target_speed = from_space_i(
            self.motor_left_target)
        self.thymio.right_wheel_target_speed = from_space_i(
            self.motor_right_target)
        self.call_leds_circle(*self.leds_circle)
        self.call_leds_top(*self.leds_top)
        self.call_leds_bottom_left(*self.leds_bottom_left)
        self.call_leds_bottom_right(*self.leds_bottom_right)

    def update(self) -> None:
        """
        Updates the Aseba variables. Should be called before
        reading them in a control step.
        """
        self.temperature = 200
        self.button_backward = int(
            self.thymio.buttons[Thymio2.Button.BACKWARD.value])
        self.button_left = int(self.thymio.buttons[Thymio2.Button.LEFT.value])
        self.button_center = int(
            self.thymio.buttons[Thymio2.Button.FORWARD.value])
        self.button_forward = int(
            self.thymio.buttons[Thymio2.Button.FORWARD.value])
        self.button_right = int(
            self.thymio.buttons[Thymio2.Button.RIGHT.value])
        self.prox_horizontal = [int(round(x)) for x in self.thymio.prox_values]

        # ################# PROX COMM RX ####################
        # The Aseba VM keeps only the current
        # event in memory and asynchronously call an callback

        events = self.thymio.prox_comm_events
        if events:
            self.prox_comm_rx__payloads = [
                smod(round(x), 16) for x in events[-1].payloads
            ]
            self.prox_comm_rx__intensities = [
                smod(round(x), 16) for x in events[-1].intensities
            ]
            self.prox_comm_rx = smod(events[-1].rx_value, 16)
        # To satisfy the ThymioAsebaProtocol,
        # we group in :py:attr:`prox_comm_buffer` all events recorded
        # in the last update step as a list of ``(rx, intensity)``.

        self._prox_comm_buffer = [(smod(event.rx_value, 16),
                                   [round(x) for x in event.intensities])
                                  for event in events]

        # ###################################################

        self.prox_ground_ambiant = [0, 0]
        self.prox_ground_reflected = to_list(self.thymio.ground_values)
        self.prox_ground_delta = self.prox_ground_reflected[:]

        self.motor_left_speed = to_space_i(
            self.thymio.left_wheel_encoder_speed)
        self.motor_right_speed = to_space_i(
            self.thymio.right_wheel_encoder_speed)
        self.motor_left_pwm = 0
        self.motor_right_pwm = 0

        self.acc = [to_acc_i(c) for c in (0, 0, 981)]
        self.acc__tap = 0
        self.r5_address = 0
        self.r5_command = 0
        self.mic_intensity = 0
        self.mic__mean = 0

    # ASEBA FUNCTIONS

    def call__leds_set(self, *args: int) -> None:
        # TODO
        pass

    def call_leds_circle(self, *values: int) -> None:
        if len(values) != 8:
            raise ValueError("Requires 8 values")
        self.thymio.leds_circle = [from_led_i(value) for value in values]
        self.leds_circle = list(values)

    def call_leds_top(self, *values: int) -> None:
        if len(values) != 3:
            raise ValueError("Requires 3 values")
        self.thymio.set_led_top(*[from_led_i(x) for x in values])
        self.leds_top = list(values)

    def call_leds_bottom_left(self, *values: int) -> None:
        if len(values) != 3:
            raise ValueError("Requires 3 values")
        self.thymio.set_led_bottom_left(*[from_led_i(x) for x in values])
        self.leds_bottom_left = list(values)

    def call_leds_bottom_right(self, *values: int) -> None:
        if len(values) != 3:
            raise ValueError("Requires 3 values")
        self.thymio.set_led_bottom_right(*[from_led_i(x) for x in values])
        self.leds_bottom_right = list(values)

    def call_leds_buttons(self, *values: int) -> None:
        if len(values) != 4:
            raise ValueError("Requires 4 values")
        self.thymio.leds_buttons = [from_led_i(value) for value in values]

    def call_leds_prox_h(self, *values: int) -> None:
        if len(values) != 8:
            raise ValueError("Requires 8 values")
        self.thymio.leds_prox = [from_led_i(value) for value in values]

    def call_leds_prox_v(self, *values: int) -> None:
        warnings.warn("LEDs near ground sensors are not simulated",
                      stacklevel=2)

    def call_leds_rc(self, *values: int) -> None:
        if len(values) != 1:
            raise ValueError("Requires one value")
        self.thymio.led_right_red = values[0]

    def call_leds_sound(self, *values: int) -> None:
        if len(values) != 1:
            raise ValueError("Requires one value")
        self.thymio.led_right_blue = values[0]

    def call_leds_temperature(self, *values: int) -> None:
        if len(values) != 2:
            raise ValueError("Requires three values")
        self.thymio.led_left_red = values[0]
        self.thymio.led_left_blue = values[1]

    def call_prox_comm_enable(self, *values: int) -> None:
        if len(values) != 1:
            raise ValueError("Requires one value")
        self.thymio.prox_comm_enabled = bool(values[0])

    def call_sound_duration(self, *args: int) -> None:
        ...

    def call_sound_freq(self, *args: int) -> None:
        ...

    def call_sound_play(self, *args: int) -> None:
        ...

    def call_sound_record(self, *args: int) -> None:
        ...

    def call_sound_replay(self, *args: int) -> None:
        ...

    def call_sound_system(self, *args: int) -> None:
        ...

    def apply(self, control: Callback[P], *args: P.args,
              **kwargs: P.kwargs) -> None:
        self.update()
        control(self, *args, **kwargs)
        self.actuate()

    def make_controller(self, behavior: Behavior) -> Controller:

        def control(obj: PhysicalObject, dt: SupportsFloat) -> None:
            """Executes the behavior"""
            assert obj is self.thymio
            self.apply(behavior, float(dt))

        return control

    def set_behavior(self, behavior: Behavior) -> None:
        self.thymio.control_step_callback = self.make_controller(behavior)
