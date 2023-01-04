# gaja_software package

This package contains software for controlling the systems of Gaja rover, as well as the legacy software in `legacy_software` directory.

## Usage

### Launch files

- `gaja_driver.launch`

  ```bash
  $ roslaunch gaja_software gaja_driver.launch
  ```

  Launches the gaja driver node.

### Nodes

- `gaja_driver`
  
  Description:

  Sends serial commands to each of the robot's wheels based on received `geometry_msgs/Twist` messages.

  Parameters:

    - `~port_name`
        - Type:         `string`
        - Default:      `/dev/ttyUSB0`
    
    Name of port on which motor driver is connected.

    - `~baudrate`
        - Type:         `int`
        - Default:      `9600`
    
    Baudrate of serial communication with motor driver.

    - `~command_timeout`
        - Type:         `float`
        - Default:      `0.5`
    
    Time, after which a message becomes obsolete. If no messages are received since that one, the robot stops.

    - `~l_x`
        - Type:         `float`
        - Default:      `0.13`

    Half of the distance between the left and the right wheels (center to center).

    - `~l_y`
        - Type:         `float`
        - Default:      `0.08`
    
    Half of the distance between the front and the rear wheels (axis to axis)

    - `~r`
        - Type:         `float`
        - Default:      `0.045`
    
    Radius of the wheel (from wheel's axis to roller's axis)
    
  Subscribed topics:
    
    - `/cmd_vel`

    Velocity commands for the rover.
