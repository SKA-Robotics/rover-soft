# sirius_cameras package
Nodes required for the operation of the cameras on the Sirius rover. Package provides:
- `oak_d_w` node - node for operating the OAK-D camera
- kalibr calibration files for the oak-d w camera
- morb_slam configuration files for the oak-d w camera and gazebo simulation
- Launch files for the `oak_d_w` node and for morb_slam

## Usage

### Launch files
- `oak_d_w.launch`
  ```bash
  roslaunch sirius_cameras oak_d_w.launch
  ```
  Launches `oak_d_w` node. Topics:
  - `~rgb/image_raw` - raw (unrectified) RGB image
  - `~depth/image_raw` - depth image (values in mm)
  - `~depth/image` - depth image (values in m)
  - `~depth/points` - colored point cloud
  - `~imu/data` - Calibrated accelerometer and gyroscope data
  Arguments:
  -  `name`
      - Type: `string`
      - Default:  `oak_d_w`  
      Name of the `oak_d_w` node
  -  `enableRviz`
      - Type:     `boolean`
      - Default:  `true`  
      Enables Rviz visualization
  
  Also all `oak_d_w` node parameters can be set with launch file arguments. For example:
  ```bash
  roslaunch sirius_cameras oak_d_w.launch fps:=20
  ```
- `morb_slam.launch`
  ```bash
  roslaunch sirius_cameras morb_slam.launch
  ```
  Launches `morb_slam` node. 
    - Subscribed topics:
      - `/oak_d_w/left/image_raw` - raw (unrectified) left camera image
      - `/oak_d_w/right/image_raw` - raw (unrectified) right camera image
      - `/oak_d_w/imu/data` - Accelerometer and gyroscope data
    - Published topics:
      - `/morb_slam_ros/camera_pose` (geometry_msgs/PoseStamped) - camera pose in the map frame
      - `/morb_slam_ros/map_points`  (sensor_msgs::PointCloud2) - tracked keypoints
      - `/morb_slam_ros/trajectory`  (nav_msgs/Path) - camera trajectory  
      
  Arguments:
    -  `camera_name`
        - Type:     `string`
        - Default:  `oak_d_w`  
        Name of the camera node (used as topic prefix)
    - `gazebo`
        - Type:     `boolean`
        - Default:  `false`  
        Remaps topics to the gazebo simulation topics and loads morb_slam configuration for the simulation
### Nodes
- `oak_d_w`
  ```bash
  $ rosrun sirius_cameras oak_d_w
  ```
  Node for operating the OAK-D camera. Topics:
  - `~rgb/image_raw` - raw (unrectified) RGB image
  - `~depth/image_raw` - depth image (values in mm)
  - `~imu/data` - Calibrated accelerometer and gyroscope data
  Parameters:
  -  `mxId`
      - Type:     `string`
      - Default:  `19443010B1B2F51200`
      - Unit:     `ms`  
      Sets the serial number of the camera to be used. If set to empty string, the first camera found will be used. For convenience, the serial number of the currently used Sirius 2 camera is set as default.
  -  `imu_frequency`
      - Type:     `integer`
      - Default:  `200`
      - Unit:     `Hz`  
      Sets the frequency of the IMU data stream. It is recommended not to set this value higher than 200 Hz, as the IMU has problems with higher frequencies for calibrated data.
  -  `fps`
      - Type:     `integer`
      - Default:  `30`
      - Unit:     `Hz`  
      Sets the frame rate of the OAK-D camera for all streams (left, right, rgb and depth). Camera support more than 30 fps, but may have problems with higher values when all streams are enabled.
  - `monoResolution`
      - Type:     `string`
      - Default:  `720p`  
      Sets the resolution of the left and right camera streams. Valid values are: 
        - `800p` (1280 x 800)
        - `720p` (1280 x 720)
        - `480p` (640 x 480)
        - `400p` (640 x 400)
  - `rgbResolution`
      - Type:     `string`
      - Default:  `1080p`  
      Sets the resolution of the rgb camera stream. Valid values are: 
        - `1080p` (1920 x 1080)
        - `4K` (3840 x 2160)
        - `12MP` (4056 x 3040)
        - `13MP` (4208 x 3120)
  - `angularVelCovariance`
      - Type:     `double`
      - Default:  `0`  
      Sets the covariance of the angular velocity measurement. 
  - `linearAccelCovariance`
      - Type:     `double`
      - Default:  `0` 
      Sets the covariance of the linear acceleration measurement.
