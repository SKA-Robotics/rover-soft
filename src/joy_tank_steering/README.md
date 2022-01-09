# joy_tank_steering package

This package allows steering the rover using a standard XBOX joystick.

- Right stick left - turn left
- Right stick right - turn right
- Right stick up - go forward
- Right stick down - go back

In case of lost signal, the rover stops.

By default, the joystick should be connected to `/dev/input/joy0`. Otherwise, the `dev` parameter in `./launch/joy_tank_steering.launch` should be changed to the appropriate path.

Maximal angular and linear velocity is set to 1 by default, but it may be changed in `./launch/joy_tank_steering.launch`.

Values of velocity are published in topic `cmd_vel` which can be read by e.g. `diff_driver_package`.