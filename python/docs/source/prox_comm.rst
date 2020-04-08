Proximity communication
=======================

A Thymio can enable proximity communication and transmits an integer payload every 0.1 s.
The message is received by other Thymios if they have enabled proximity communication too and if at least one of their proximity sensors
is visible by one of the emitter's [proximity] LEDs.

Run-time
~~~~~~~~~

At each control step, the simulation collect neighbors' messages sent after the last control step
into :attr:`pyenki.Thymio2.prox_comm_events`.

Note that each message is broadcasted only for 1 control step.
Therefore if the duration of a control step (i.e. the period at which messages are received)
is shorter than 0.1 (i.e. the period at which messages are sent),
for some control steps the robot will receive no message.

This mimic the behavior of the real robot that will receive short pulses of IR light sent by a neighbor every 0.1 seconds.
One minor difference is that the real robots tries to avoid collision between their IR pulses by moving
slightly the timing of their emission in the 0.1 second long time-window. On the contrary, the simulated robots,
sent the messages immediately before the next control step and their messages do not incur into collisions.



Simulated Response
~~~~~~~~~~~~~~~~~~

More precisely, given an emitting [proximity] LED at point :math:`p_e` and angle :math:`\alpha_e` and a receiving [proximity] sensor at point :math:`p_r` and angle :math:`\alpha_r`,
a pulse from `e` to `r` is visible if the following criteria are met:

1) the distance between :math:`p_e` and :math:`p_r` is less then  :math:`\rho` ~ 23 cm,
2) `p_r` is inside the circular sector [:math:`\alpha_e` - :math:`\beta_e`, :math:`\alpha_e` + :math:`\beta_e`] centered at :math:`p_e`, with :math:`\beta_e` ~ 15 degrees (0.268 rad),
3) `p_e` is inside the circular sector [:math:`\alpha_r` - :math:`\beta_r`, :math:`\alpha_r` + :math:`\beta_r`] centered at :math:`p_r`, with :math:`\beta_r` ~ 36 degrees (0.644 rad),
4) there are no obstacles along the segment :math:`\overline{p_r p_e}`.

The light intensities of all emitters :math:`e` are summed up and stimulate a noisy response :math:`r` :

.. math::

  r \approx \frac{m}{\frac{1}{\sum_e \frac{c-x_0^2}{(|p_r - p_e|-x_0)^2}} + 1}

where :math:`r` ~ 4200 is the maximal value, :math:`x_0` ~ 0.02 cm is a small offset and `c` ~ 275 :math:`\textrm{cm}^2` is a bias

We set to 0 all responses not large enough (i.e., lower than the response caused by an emitter at distance :math:`\rho`).
Note that this is still a simplification because we ignore that the real intensities depends also on the orientations.


Each received message in encoded as an :ref:`IRCommEvent` ``event``, where ``event.payloads`` and ``event.intensities`` contain the response of the seven proximity sensors:
the first five entries are from frontal sensors ordered from left to right and the last two entries are from rear sensors ordered from left to right.
When a sensor doesn't receive the message (i.e., when the response is too weak), the corresponding payload and intensity are set to zero.
