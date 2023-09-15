# panel_visualization package

This package provides visualization of panel used in maintance task in European Rover Chalenge by detection of aruco tags. It also allows to track any other object, which posses aruco markers on its surface. Mesh models of all configured objects are published as Rviz markers. Their poses are also broadcasted via TF servis.

## Usage

### Launch files

- `aruco_detection.launch`

    ```bash
    roslaunch panel_visualization aruco_detection.launch
    ```

    Runs only `aruco_detect` node, which publishes marker position obtained by one camera stream.

- `panel_visualization.launch`

    ```bash
    roslaunch panel_visualization aruco_detection.launch
    ```

    Runs `panel_tracker` and up to 6 instances of `aruco_detect` node. Publishes objects position obtained by averaging data from all connected camera streams.

### Nodes

- `panel_tracker`

    Gets data from each `/fiducial_transforms` topic, calculates position of each configured object and publish it simultaneously with corresponding Rviz markers.

    Parameters:

  - `~base_frame`
    - Type: `string`
    - Default: `base_link`

    Parent frame to `/tf` publishing. When it is a one of the visual objects, cameras frames do not have to be known.

  - `~cameras_ids`
    - Type: `string`
    - Default: `[]`

    ID's associated to name spaces `/web_camera_X/webcam` for image transport written in list format. Each one should have an appropriate `aruco_detect` node.

  - `~message_rate`
    - Type: `double`
    - Default: `1.0`

    Set marker life time, and fake stream speed in case of test mode.

  - `~filter_size`
    - Type: `int`
    - Default: `3`

    Number of frames used to averaging visual objects pose.

  - `~marker_length`
    - Type: `double`
    - Default: `0.05`
    - Unit: `meters`

    Physical length of black part of aruco tags.

  - `~marker_lengths_override`
    - Type: `string`
    - Default: `""`

    Tags IDs with lengths different than others. Written in format compatible with `aruco_detect` node parameter.
  
- `aruco_detect`

    Detects aruco markers and published data via `/fiducial_transforms` and many other topics. For more informations see <https://wiki.ros.org/aruco_detect>.

    Parameters:

  - `~dictionary`
    - Type: `int`
    - Default: `7`

  - `~publish_images`
    - Type: `bool`
    - Default: `false`

  - `~fiducial_len`
    - Type: `double`
    - Default: `0.14`

  - `~fiducial_len_override`
    - Type: `string`
    - Default: `""`

### Config files

- `visual_objects.yaml`

    Contains a list of objects with their published names, mesh files paths and poses of each tag in reference frame suitable to the mesh object.

## Topics

### Subscribed topics

- `<camera_ns>/camera_info` [sensor_msgs/CameraInfo]
- `<camera_ns>/image_raw` [sensor_msgs/Image]

    Each instance gets video stream from a camera by topics (similar to image transport convention) with a namespace *<camera_ns>* provided by a parameter.

- `/tf` [geometry_msgs/TransformStamped]
- `/tf_static` [geometry_msgs/TransformStamped]

### Published topics

- `/visual_objects` [visualization_msgs/MarkerArray]

    Publishes markers with mesh models of each detected object.

- `/tf` [geometry_msgs/TransformStamped]

    Publishes transforms from base frame to reference frame of mesh model of each detected object.
