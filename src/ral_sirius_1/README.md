# ral_sirius_1 package
Rover Abstraction Layer for Sirius 1 rover. Package provides:
- manipulator control
- status light control

## Usage

### Launch files
- `manip_control.launch`
  ```bash
  $ roslaunch ral_sirius_1 manip_control.launch
  ```
  Launch manipulator control module.

### Configs
- `manip_control.yaml`
  Serial port and baudrate for manipulator driver must be specified here.
  You can also change movement speeds of individual manipulator limbs.

## Topics

### Subscribed topics
- `/cmd_manip` [sensor_msgs/JointState]
  Sets target position, velocity and effort of individual manipulator limbs.

  Each limb can be specified using its name:
  - `arm_rotate` - rotation of entire arm
  - `claw_clamp` - extending of claw jaws
  - `claw_rotate` - rotation of claw
  - `claw_lift` - pitching of claw
  - `arm_tilt` - raising/lowering end-half of arm
  - `arm_lift` - raising/lowering of entire arm

  Each limb should be specified at most once. If a limb should't move
  it can be ommited from the message.

  ```{caution}
  Currently only effort is supported, which should range between -1.0 and 1.0, meaning
  how fast should a limb move in specified direction (1.0 is max configured speed).

  To be improved™ when we get encoders for manipulator arm motors.
  ```
