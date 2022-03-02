# joy_tank_steering package

This package allows steering the rover and the manipulator using one or more standard XBOX wireless joysticks.

Steering can be switched between rover and manipulator using arrow buttons.

## Usage

### Launch files

- `multi_joy_steering.launch`

  ```bash
  $ roslaunch joy_tank_steering multi_joy_steering.launch
  ```

  Launch multiple joysticks steering module.

### Nodes

- `joy_node` (one or more with indexes)

  Parameters:

  - `~dev`
    - Type:       `string`
    - Dafault:    `/dev/input/js0`
  - `~deadzone`
    - Type:       `double`
    - Default:    `0.05`
    - Set:        `0.15`
  - `~coalesce_interval`
    - Type:       `double`
    - Default:    `0.001`
    - Set:        `0.02`
    - Unit:       `sec`
  - `~autorepeat_rate`
    - Type:       `double`
    - Default:    `0.0`
    - Set:        `50.0`
    - Unit:       `Hz`
- `joystick_steering_node` (one or more with indexes)

  Parameters:

  - `~joy_message_topic_name`
    - Type:       `string`
    - Dafault:    `joy`

    Topic to subscribe messages from each node.
  - `~start_steering_mode`
    - Type:       `int`
    - Default:    `0`

    ROVER_MODE = 0, MANIPULATOR_MODE = 1
  - `~tank_steering_mode`
    - Type:       `bool`
    - Default:    `True`
    - Set:        `False`

    Enable steering each side of rover (left or right) by one stick.
  - `~ID`
    - Type:       `int`
    - Default:    `0`

    Identification number of each joystick and corresponding node.

## Topics

### Subscribed topics

- `/joy` [sensor_msgs/Joy]
  Gets position of individual joystick axes and buttons.

### Published topics

- `/cmd_vel` [geometry_msgs/Twist]
  Sets linear and angular velocity of rover.
- `/cmd_manip` [sensor_msgs/JointState]
  Sets target position, velocity and effort of individual manipulator limbs.

## Steering description

### Rover steering

In normal mode:

- Right stick left/right - turn left/right
- Right stick up/down - go forward/back

In tank mode:

- Left stick up/down - let left weels go forward/back
- Right stick up/down - let right weels go forward/back

This mode can be changed by parameter `tank_steering_mode` in file `./launch/joy_tank_steering.launch`.

### Manipulator steering

- Left stick up/down - lift arm
- Left stick left/right - rotate arm
- Right stick up/down - lift claw
- Right stick left/right - rotate claw
- Back axis on left/right - tilt arm
- Back buttons on left/right - clap claw

In case of lost signal, the rover or the manipualtor stops.
