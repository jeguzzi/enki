from typing import ClassVar, List, Tuple, overload

Vector = Tuple[float, float]


class Color:
    """
        An RGBA color with values between 0.0 and 1.0.

        Args:
            r (float): Red channel, in [0, 1], optional (default 0.0)
            g (float): Green channel, in [0, 1], optional (default 0.0)
            b (float): Blue channel, in [0, 1], optional (default 0.0)
            a (float): Alpha channel, in [0, 1], optional (default 1.0)

        Attributes:
            black (Color): Black color (*readonly*)
            gray (Color) : Gray color (*readonly*)
            white (Color) : White color (*readonly*)
            red (Color) : Red color (*readonly*)
            green (Color) : Green color (*readonly*)
            blue (Color) : Blue color (*readonly*)
            r (float): Red channel, in [0, 1]
            g (float): Green channel, in [0, 1]
            b (float): Blue channel, in [0, 1]
            a (float): Alpha channel, in [0, 1]
            components (Tuple[float, float, float, float]): Components, in [0, 1]
    """

    # TODO: mark them as read-only [class] properties
    black: ClassVar['Color'] = None
    white: ClassVar['Color'] = None
    gray: ClassVar['Color'] = None
    red: ClassVar['Color'] = None
    green: ClassVar['Color'] = None
    blue: ClassVar['Color'] = None

    r: float
    g: float
    b: float
    a: float
    components: Tuple[float, float, float, float]

    def __init__(self, r: float = 0.0, g: float = 0.0, b: float = 0.0, a: float = 1.0) -> None:
        ...


class PhysicalObject:
    """
        The superclass of objects that can be simulated.

        Attributes:
            position (Tuple[float, float]) : The position in the world frame in centimeters
            angle (float): The orientation in the world frame in radians
            velocity (Tuple[float, float]): The velocity in the world frame in centimeters per second
            angular_speed (float): The angular speed in the world frame in radians per second

            radius (float): The radius of the object's enclosing circle in centimeters
            height (float): The object height in centimeters
            is_cylindric (bool): True if the object is cylindrical shaped.
            mass (float): The object mass in kilograms. If below zero, the object can't move (infinite mass).
            moment_of_inertia (float): The obejct moment of inertial [Note: this is a symmetrical simplification.]

            color (Color): The object color.
            collision_elasticity (float): Elasticity of collisions of this object. If 0, soft collision, 100% energy dissipation; if 1, elastic collision, 0% energy dissipation. Actual elasticity is the product of the elasticity of the two colliding objects. Walls are fully elastics
            dry_friction_coefficient (float): The dry friction coefficient mu
            viscous_friction_coefficient (float): The viscous friction coefficient. Premultiplied by mass. A value of k applies a force of -k * speed * mass
            viscous_moment_friction_coefficient (float):
    """

    collision_elasticity: float
    dry_friction_coefficient: float
    viscous_friction_coefficient: float
    viscous_moment_friction_coefficient: float
    position: Vector
    angle: float
    velocity: Vector
    angular_speed: float
    color: Color

    @property
    def radius(self) -> float:
        ...

    @property
    def height(self) -> float:
        ...

    @property
    def is_cylindric(self) -> bool:
        ...

    @property
    def mass(self) -> float:
        ...

    @property
    def moment_of_inertia(self) -> float:
        ...


class IRCommEvent:
    """
        An `IRCommEvent` is returned each time a message is received by at least one proximity sensor.
        The sensors that do not receive the message, have the corresponding payloads and intensities set to zero.

        Attributes:
            rx (int): The received message payload (*readonly*)
            payloads (List[int]): A list of 7 payloads, one for each sensors (*readonly*)
            intensities (List[int]): A list of 7 intensities, one for each sensors (*readonly*)
    """
    @property
    def rx(self) -> int:
        ...

    @property
    def payloads(self) -> List[int]:
        ...

    @property
    def intensities(self) -> List[int]:
        ...


class Thymio2(PhysicalObject):
    """
        Create a `PhysicalObject` Thymio2 robot.
        Attribute names mimic the aseba interface, see http://wiki.thymio.org/en:thymioapi

        Args:
            use_aseba_units (bool): use the same units and type as Aseba (default `False`).

        Example (non aseba units):
        ::
            >>> import pyenki
            >>> thymio = pyenki.Thymio2(use_aseba_units=False)
            >>> thymio.position = (3.2, 5.6)
            >>> thymio.angle = 1.3
            >>> # Spin the robot on itself
            >>> thymio.motor_left_target = 10.2
            >>> thymio.motor_right_target = -10.2
            >>> # Switch the top LED yellow
            >>> thymio.set_led_top(0.5, 0.5, 0.0)
            >>> thymio.prox_values
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        Example (equivalent with aseba units):
        ::
            >>> import pyenki
            >>> thymio = pyenki.Thymio2(use_aseba_units=True)
            >>> thymio.position = (3.2, 5.6)
            >>> thymio.angle = 1.3
            >>> # Spin the robot on itself
            >>> thymio.motor_left_target = 307
            >>> thymio.motor_right_target = -307
            >>> # Switch the top LED yellow
            >>> thymio.set_led_top(15, 15, 0)
            >>> thymio.prox_values
            [0, 0, 0, 0, 0, 0, 0]

        Attributes:
            use_aseba_units (bool): whenever the robot should use the same units and type as the Aseba interface.

                When ``use_aseba_units=False``, the robot
                    - uses floating point numbers,
                    - measures the space in centimeters
                    - normalizes color values between 0.0 and 1.0.
                Instead, when ``use_aseba_units=True``, the robot
                    - uses integers numbers,
                    - measures the space in discrete steps such that 500 steps ~= 16.6 cm,
                    - normalizes (LED) color values in between 0 and 31.

            prox_values (List[number]): A list of (7) proximity sensor readings, one for each sensors (*readonly*)
            prox_distances (List[float]): A list of (7) distances between proximity sensor and nearest obstancle, one for each sensors;
                please note that this variable is directly accessible by the real robot (*readonly*)
            prox_comm_tx (int): The integer payload to be sent. The real robot can only send 11 bits,
                therefore to be compliant we should limit the value between 0 and 2047.
            prox_comm_enable (bool): Enable/disable proximity communication.
            prox_comm_events (List[IRCommEvent]): A list of events, one for every received message during the last control step.
            ground_values (List[number]): A list of (2) ground sensor readings, one for each sensors (*readonly*)
            motor_{left,right}_target (number): The target {left,right} wheel speed
            motor_{left,right}_speed (number): The {left,right} wheel speed measured by simulated wheel encoders (*readonly*)
            motor_{left,right}_odometry (number): The {left,right} wheel odometry intergrated from measured wheel speeds.
    """

    prox_comm_tx: int
    prox_comm_enable: bool
    motor_left_target: float
    motor_right_target: float
    use_aseba_units: bool

    def __init__(self, use_aseba_units: bool = False) -> None:
        ...

    def controlStep(self, dt: float) -> None:
        """
            Perform one control step.
            This method should be overwritten by any subclass to control the robot.

            Args:
                dt (float): The control step duration.
        """
        ...

    @property
    def prox_values(self) -> List[float]:
        ...

    @property
    def prox_distances(self) -> List[float]:
        ...

    @property
    def ground_values(self) -> List[float]:
        ...

    @property
    def prox_comm_events(self) -> List[IRCommEvent]:
        ...

    def set_led_top(self, red: float = 0, green: float = 0, blue: float = 0) -> None:
        """
            Control the top RGB LED color

            Args:
                red (number): the value of the red channel
                green (number): the value of the green channel
                blue (number): the value of the blue channel
        """
        ...

    def set_led_bottom_left(self, red: float = 0, green: float = 0, blue: float = 0) -> None:
        """
            Control the bottom-left RGB LED color

            Args:
                red (number): the value of the red channel
                green (number): the value of the green channel
                blue (number): the value of the blue channel
        """
        ...

    def set_led_bottom_right(self, red: float = 0, green: float = 0, blue: float = 0) -> None:
        """
            Control the bottom-right RGB LED color

            Args:
                red (number): the value of the red channel
                green (number): the value of the green channel
                blue (number): the value of the blue channel
        """
        ...

    def set_led_buttons(self, index: int, value: float) -> None:
        """
            Control one of the 4 red LED near to the arrow buttons.

            Args:
                index (int): the index of the led (between 0 and 3)
                value (number): the intensity of the color.
        """
        ...

    def set_led_prox(self, index: int, value: float) -> None:
        """
            Control one of the 8 red LED near to the proximity sensors.

            Args:
                index (int): the index of the led (between 0 and 7)
                value (number): the intensity of the color.
        """
        ...

    def set_led_circle(self, index: int, value: float) -> None:
        """
            Control one of the 8 yellow LED that forms a circle on top of the robot.

            Args:
                index (int): the index of the led (between 0 and 7)
                value (number): the intensity of the color.
        """
        ...

    def set_led_left_red(self, value: float) -> None:
        """
            Control the red LED on the left side.

            Args:
                value (number): the intensity of the color.
        """
        ...

    def set_led_right_red(self, value: float) -> None:
        """
            Control the red LED on the right side.

            Args:
                value (number): the intensity of the color.
        """

    def set_led_left_blue(self, value: float) -> None:
        """
            Control the blue LED on the left side.

            Args:
                value (number): the intensity of the color.
        """
        ...

    def set_led_right_blue(self, value: float) -> None:
        """
            Control the blue LED on the right side.

            Args:
                value (number): the intensity of the color.
        """
        ...

    @property
    def motor_left_speed(self) -> float:
        ...

    @property
    def motor_right_speed(self) -> float:
        ...

    @property
    def motor_left_odometry(self) -> float:
        ...

    @property
    def motor_right_odometry(self) -> float:
        ...


class CircularObject(PhysicalObject):
    """
        Create a `PhysicalObject` cylinder with a given mass and color.

        Args:
            radius (float): Radius in centimeters
            height (float): Height in centimeters
            mass (float): Mass in kilograms
            color (Color): Color
    """

    def __init__(self, radius: float, height: float, mass: float,
                 color: Color = Color.black) -> None:
        ...


class RectangularObject(PhysicalObject):
    """
        Create a `PhysicalObject` box with a given mass and color.

        Args:
            l1 (float): Depth in centimeters
            l2 (float): Width in centimeters
            height (float): Height in centimeters
            mass (float): Mass in kilograms
            color (Color): Color
    """

    def __init__(self, l1: float, l2: float, height: float, mass: float,
                 color: Color = Color.black) -> None:
        ...


class World:
    """
        The world is the container of all objects and robots.
        It is either
            - a rectangular arena with walls at all sides:
              ::
                World(width: float, height: float, walls_color: Color = Color.gray)

            - a circular area with walls:
              ::
                World(width: float, height: float, walls_color: Color = Color.gray)

            - or an infinite surface:
              ::
                World()

        Args:
            width (float): The rectangular world width in centimeters
            height (float): The rectangular world height in centimeters
            radius (float): The circular world radius in centimeters
            walls_color (Color): Optional wall color, default is ``Color.gray``

        Example:
        ::
            import pyenki

            world = pyenki.World()
            thymio = Thymio2()
            world.add_object(thymio)
            wall = pyenki.RectangularObject(l1=10, l2=50, height=5, mass=1,
                                            color=pyenki.Color(0.5, 0.3, 0.3))
            world.add_object(wall)
            # Run 100 times a 0.1 s long simulation step
            for _ in range(100):
                world.step(0.1)

    """

    @overload
    def __init__(self, width: float, height: float, walls_color: Color = Color.gray) -> None:
        ...

    @overload  # noqa: F811
    def __init__(self, r: float, walls_color: Color = Color.gray) -> None:
        ...

    @overload  # noqa: F811
    def __init__(self) -> None:
        ...

    def step(self, dt: float, physics_oversampling: int = 1) -> None:
        """
            Simulate a timestep

            Args:
                dt (float): the update timestep in seconds, should be below 1 (typically .02-.1)
                physics_oversampling (int): the amount of time the physics is run per step,
                as usual collisions require a more precise simulation than the sensor-motor loop frequency
        """
        ...

    def add_object(self, item: PhysicalObject) -> None:
        """
            Add a physical object to the simulation.

            Args:
                item (PhysicalObject): the object to add.
        """
        ...

    def remove_object(self, item: PhysicalObject) -> None:
        """
            Remove a physical object from the simulation.

            Args:
                item (PhysicalObject): the object to remove.
        """
        ...

    def set_random_seed(self, seed: int) -> None:
        """
            Set the random seed

            Note: Not sure that it is working as expected.
        """
        ...

    def run(self, steps: int) -> None:
        """
            Run for a number of steps, using a step of 1/30 s and `physics_oversampling` of 3.

            Args:
                steps (int): The number of steps to perform
        """
        ...

    def run_in_viewer(self, cam_position: Vector = (0, 0), cam_altitude: float = 0,
                      cam_yaw: float = 0, cam_pitch: float = 0, walls_height: float = 10,
                      orthographic: bool = False) -> None:
        """
            Launch a QApplication to visualize the world from a (camera) viewpoint.
            The application run loop will keep real time while calling `World.step`
            at the appropriate time.

            Args:
                cam_position (Tuple[float, float]): the position of the viewpoint in centimeters
                cam_altitude (float): the altitude of the viewpoint in centimeters
                cam_yaw (float): the yaw of the viewpoint in radians
                cam_pitch (float): the pitch of the viewpoint in radians
                walls_height (float): the height of (visualized) walls in centimeters
                orthographic (bool): whetever to use a down looking camera with ortographic projection (default `False`)
        """
        ...


class WorldView:
    """
        A QWidget subclass to display the world that automatically refreshes 30 times per second.
        It requires that a QApplication has already been instantiated.

        Args:
            world (World): the world to be displayed
            run_world_update (bool): whetever to run or not ``world.step`` at each view refresh (default `False`)
            cam_position (Tuple[float, float]): the position of the viewpoint in centimeters (default `(0, 0)`)
            cam_altitude (float): the altitude of the viewpoint in centimeters (default `0`)
            cam_yaw (float): the yaw of the viewpoint in radians (default `0`)
            cam_pitch (float): the pitch of the viewpoint in radians (default `0`)
            walls_height (float): the height of world boundary walls in centimeters (default `10`)
            orthographic (bool): whetever to use a down looking camera with ortographic projection (default `False`)

        Attributes:
            run_world_update (bool): whetever to run or not ``world.step`` at each view refresh.
            cam_position (Tuple[float, float]): the position of the viewpoint in centimeters
            cam_altitude (float): the altitude of the viewpoint in centimeters
            cam_yaw (float): the yaw of the viewpoint in radians
            cam_pitch (float): the pitch of the viewpoint in radians
            orthographic (bool): whetever to use a down looking camera with ortographic projection. If enabled, `cam_pitch` is fixed to :math:`-\pi/2`.

        Example (in an IPython qtconsole/notebook):
        ::
            >>> import pyenki
            >>> world = pyenki.World()
            >>> %gui qt5
            >>> view = pyenki.WorldView()
            >>> view.show()
    """

    run_world_update: bool
    cam_position: Tuple[float, float]
    cam_altitude: float
    cam_yaw: float
    cam_pitch: float
    orthographic: bool

    def __init__(self, world: World, run_world_update: bool = False,
                 cam_position: Tuple[float, float] = (0, 0), cam_altitude: float = 0,
                 cam_yaw: float = 0, cam_pitch: float = 0, walls_height: float = 10,
                 orthographic: bool = False) -> None:
        ...

    def show(self) -> None:
        """
            Show the view
        """
        ...

    def hide(self) -> None:
        """
            Hide the view
        """
        ...
