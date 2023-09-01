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

    Runs `panel_tracker` and a few instances of `aruco_detect` node. Publishes objects position obtained by averaging data from all connected camera streams.

### Nodes

- `panel_tracker`

    Gets data from each `/fiducial_transforms` topic, calculates position of each configured object and publish it simultaneously with corresponding Rviz markers.

    Parameters:

  - `~base_frame`
  - `~cameras_names`
  - `~message_rate`
  - `~filter_size`
  - `~marker_length`
  - `~marker_lengths_override`
  
- `aruco_detect`

    Detects aruco markers and published data via `/fiducial_transforms` and many other topics.

    Parameters:

  - `~dictionary`
  - `~publish_images`
  - `~fiducial_len`
  - `~fiducial_len_override`

### Config files

- `visual_objects.yaml`

    Contains a list of objects with their published names, mesh files paths and poses of each tag in reference frame suitable to the mesh object.

## Topics

### Subscribed topics

- `<camera_ns>/camera_info` [sensor_msgs/CameraInfo]
- `<camera_ns>/image` [sensor_msgs/Image]

    Each instance gets video stream from a camera by topics (similar to image transport convention) with a namespace *<camera_ns>* provided by a parameter.

- `/tf` [geometry_msgs/TransformStamped]
- `/tf_static` [geometry_msgs/TransformStamped]

### Published topics

- `/visual_objects` [visualization_msgs/MarkerArray]

    Publishes markers with mesh models of each detected object.

- `/tf` [geometry_msgs/TransformStamped]

    Publishes transforms from base frame to reference frame of mesh model of each detected object.
