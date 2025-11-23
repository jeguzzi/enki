from __future__ import annotations

import math
import warnings

from .. import Array1D, Thymio2
from .utils import smod


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
        # Use a similar API

        aseba.call_leds_top(32, 0, 0)
        if aseba.prox_horizontal[2] > 2000:
            aseba.motor_left_target = 123
        else
            aseba.motor_left_target = 0
        end

    The exception are variables related to proximity communication.
    While aseba keeps only the most-recent received
    message in ``prox.comm.rx._payloads``, ``prox.comm.rx._intensities``
    and ``prox.comm.rx`` and generate an event each time a message is received,
    this adapter returns a **list** of messages received in the last update steps,
    in :py:attr:`prox_comm_rx__payloads`, :py:attr:`prox_comm_rx__intensities`,
    and :py:attr:`prox_comm_rx`.
    """

    def __init__(self, thymio: Thymio2) -> None:
        self.thymio = thymio
        self.mic_threshold = 0
        self.time_period = [0, 0]
        self.sd_present = 0

    # ASEBA VARIABLES

    @property
    def temperature(self) -> int:
        warnings.warn("temperature is not simulated", stacklevel=2)
        return 200

    @property
    def button_backward(self) -> int:
        return int(self.thymio.buttons[Thymio2.Button.BACKWARD.value])

    @property
    def button_left(self) -> int:
        return int(self.thymio.buttons[Thymio2.Button.LEFT.value])

    @property
    def button_center(self) -> int:
        return int(self.thymio.buttons[Thymio2.Button.CENTER.value])

    @property
    def button_forward(self) -> int:
        return int(self.thymio.buttons[Thymio2.Button.FORWARD.value])

    @property
    def button_right(self) -> int:
        return int(self.thymio.buttons[Thymio2.Button.RIGHT.value])

    @property
    def prox_horizontal(self) -> list[int]:
        return [round(x) for x in self.thymio.prox_values]

    # ################# PROX COMM RX ####################
    # This is different than aseba that keeps the current
    # event in memory and asynchronously call an callback
    #
    # Here we don't use aseba callbacks and we return
    # all events recorded in the last update step.
    @property
    def prox_comm_rx__payloads(self) -> list[list[int]]:
        events = self.thymio.prox_comm_events
        return [[smod(round(x), 16) for x in event.payloads]
                for event in events]

    @property
    def prox_comm_rx__intensities(self) -> list[list[int]]:
        events = self.thymio.prox_comm_events
        return [[round(x) for x in event.intensities] for event in events]

    @property
    def prox_comm_rx(self) -> list[int]:
        events = self.thymio.prox_comm_events
        return [smod(event.rx_value, 16) for event in events]

    # ###################################################

    @property
    def prox_comm_tx(self) -> int:
        return smod(self.thymio.prox_comm_tx, 16)

    @prox_comm_tx.setter
    def prox_comm_tx(self, value: int) -> None:
        self.thymio.prox_comm_tx = smod(value, 16)

    @property
    def prox_ground_ambiant(self) -> list[int]:
        warnings.warn("ambient light is not simulated", stacklevel=2)
        return [0, 0]

    @property
    def prox_ground_reflected(self) -> list[int]:
        return self.prox_ground_delta

    @property
    def prox_ground_delta(self) -> list[int]:
        return to_list(self.thymio.ground_values)

    @property
    def motor_left_target(self) -> int:
        return to_space_i(self.thymio.left_wheel_target_speed)

    @motor_left_target.setter
    def motor_left_target(self, value: int) -> None:
        self.thymio.left_wheel_target_speed = from_space_i(value)

    @property
    def motor_right_target(self) -> int:
        return to_space_i(self.thymio.right_wheel_target_speed)

    @motor_right_target.setter
    def motor_right_target(self, value: int) -> None:
        self.thymio.right_wheel_target_speed = from_space_i(value)

    @property
    def motor_left_speed(self) -> int:
        return to_space_i(self.thymio.left_wheel_encoder_speed)

    @property
    def motor_right_speed(self) -> int:
        return to_space_i(self.thymio.right_wheel_encoder_speed)

    @property
    def motor_left_pwm(self) -> int:
        warnings.warn("motors pwm are not simulated", stacklevel=2)
        return 0

    @property
    def motor_right_pwm(self) -> int:
        warnings.warn("motors pwm are not simulated", stacklevel=2)
        return 0

    @property
    def acc(self) -> list[int]:
        warnings.warn("accelerometer is not simulated", stacklevel=2)
        return [to_acc_i(c) for c in (0, 0, 981)]

    @property
    def acc__tap(self) -> int:
        warnings.warn("tap is not simulated", stacklevel=2)
        return 0

    @property
    def r5_address(self) -> int:
        warnings.warn("IR Remote is not simulated", stacklevel=2)
        return 0

    @property
    def r5_command(self) -> int:
        warnings.warn("IR Remote is not simulated", stacklevel=2)
        return 0

    @property
    def mic_intensity(self) -> int:
        warnings.warn("Microphone is not simulated", stacklevel=2)
        return 0

    @property
    def mic__mean(self) -> int:
        warnings.warn("Microphone is not simulated", stacklevel=2)
        return 0

    @property
    def leds_top(self) -> list[int]:
        return [to_led_i(x) for x in self.thymio.get_led_top()]

    @leds_top.setter
    def leds_top(self, value: list[int]) -> None:
        if len(value) != 3:
            raise ValueError("Requires three values")
        return self.thymio.set_led_top(*[from_led_i(x) for x in value])

    @property
    def leds_bottom_left(self) -> list[int]:
        return [to_led_i(x) for x in self.thymio.get_led_bottom_left()]

    @leds_bottom_left.setter
    def leds_bottom_left(self, value: list[int]) -> None:
        if len(value) != 3:
            raise ValueError("Requires three values")
        return self.thymio.set_led_bottom_left(*[to_led_i(x) for x in value])

    @property
    def leds_bottom_right(self) -> list[int]:
        return [to_led_i(x) for x in self.thymio.get_led_bottom_right()]

    @leds_bottom_right.setter
    def leds_bottom_right(self, value: list[int]) -> None:
        if len(value) != 3:
            raise ValueError("Requires three values")
        return self.thymio.set_led_bottom_right(*[to_led_i(x) for x in value])

    @property
    def leds_circle(self) -> list[int]:
        return [to_led_i(x) for x in self.thymio.leds_circle]

    @leds_circle.setter
    def leds_circle(self, values: list[int]) -> None:
        self.thymio.leds_circle = [from_led_i(value) for value in values]

    # ASEBA FUNCTIONS

    def call_leds_circle(self, *values: int) -> None:
        self.leds_circle = list(values)

    def call_leds_top(self, *values: int) -> None:
        self.leds_top = list(values)

    def call_leds_bottom_left(self, *values: int) -> None:
        self.leds_bottom_left = list(values)

    def call_leds_bottom_right(self, *values: int) -> None:
        self.leds_bottom_right = list(values)

    def call_leds_buttons(self, *values: int) -> None:
        self.thymio.leds_buttons = [from_led_i(value) for value in values]

    def call_leds_prox_h(self, *values: int) -> None:
        self.thymio.leds_prox = [from_led_i(value) for value in values[:3] + values[2:]]

    def call_leds_prox_v(self, *values: int) -> None:
        warnings.warn("LEDs near ground sensors are not simulated",
                      stacklevel=2)

    def call_leds_rc(self, value: int) -> None:
        self.thymio.led_right_red = value

    def call_led_sound(self, value: int) -> None:
        self.thymio.led_right_blue = value

    def call_led_temperature(self, *values: int) -> None:
        if len(values) != 2:
            raise ValueError("Requires three values")
        self.thymio.led_left_red = values[0]
        self.thymio.led_left_blue = values[1]

    def call_comm_enable(self, value: int) -> None:
        self.thymio.prox_comm_enabled = bool(value)
