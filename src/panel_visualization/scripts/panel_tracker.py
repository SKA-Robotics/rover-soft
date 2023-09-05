#!/usr/bin/python3
import numpy as np
import tf.transformations as tft
from threading import Lock

import rospy
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Quaternion, Transform, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

WARNING_DURATION = 3  # [number of frames]
YOUR_RVIZ_SPEED = 1.0  # [Real FPS]


class PanelTracker:
    def __init__(self) -> None:
        rospy.init_node("panel_tracker")

        message_rate: float = rospy.get_param("~message_rate", 1.0)
        filter_size: int = rospy.get_param("~filter_size", 3)
        lifetime = max(filter_size / message_rate, 1.0 / YOUR_RVIZ_SPEED)
        self.tags_lifetime = rospy.Duration.from_sec(lifetime)

        self.base_frame: str = rospy.get_param("~base_frame", "base_link")
        objects: list[dict] = rospy.get_param("~visual_objects", {})
        self.objects = [
            VisulObject(obj_params, filter_size,
                        rospy.Duration.from_sec(lifetime))
            for obj_params in objects if obj_params['name'] != self.base_frame
        ]
        base_objs = [obj for obj in objects if obj['name'] == self.base_frame]
        self.base_object = VisulObject(
            base_objs[0], 1,
            rospy.Duration.from_sec(lifetime)) if len(base_objs) else None

        self.marker_size: float = rospy.get_param("~marker_length", 0.05)
        lengths: str = rospy.get_param("~marker_lengths_override", "")
        self.sizes = self._translate_marker_sizes(lengths)

        name_list: str = rospy.get_param("~cameras_names", "[]")
        if name_list[0] != '[' or name_list[-1] != ']':
            rospy.logwarn("Incorrect syntax for 'camera_name' param!")
            return

        for name in name_list[1:-1].split(','):
            rospy.Subscriber(f'{name.strip()}/fiducial_transforms',
                             FiducialTransformArray,
                             self._callback,
                             queue_size=10)
        self.pub = rospy.Publisher("/visual_objects",
                                   MarkerArray,
                                   queue_size=10)

        self.buffer = Buffer()
        TransformListener(self.buffer)

    def run(self) -> None:
        rospy.spin()

    def _translate_marker_sizes(self, lengths: str) -> 'dict[int, float]':
        try:
            sizes: dict[int, float] = {}
            for (k, v) in [scope.split(':') for scope in lengths.split(',')]:
                if k.find('-') != -1:
                    for (s, e) in [k.split('-')]:
                        sizes.update(
                            {i: float(v)
                             for i in range(int(s),
                                            int(e) + 1)})
                else:
                    sizes.update({int(k): float(v)})
            return sizes
        except:
            rospy.logwarn(
                "Marker length override data are incorrect! They have been ignorred."
            )
            return {}

    def _callback(self, msg: FiducialTransformArray) -> None:
        new_msg = MarkerArray()
        transforms: list[FiducialTransform] = msg.transforms
        transforms_dict = {t.fiducial_id: t for t in transforms}

        if self.base_object is None:
            try:
                transform_msg: TransformStamped = self.buffer.lookup_transform(
                    self.base_frame, msg.header.frame_id, msg.header.stamp)
                frame_transform = transform_msg.transform
            except Exception as ex:
                template = "An exception of type {0} occurred. Arguments:\n{1!r}"
                message = template.format(type(ex).__name__, ex.args)
                rospy.logwarn(message)
                return
        else:
            with self.base_object.get_lock():
                self.base_object.update_pose(transforms_dict, None)
                if self.base_object.is_ready():
                    base_header = Header(stamp=msg.header.stamp,
                                         frame_id=msg.header.frame_id)
                    marker = self.base_object.generate_next_marker(base_header)
                    if marker is not None:
                        new_msg.markers.append(marker)
                        self.base_object.publish_transform(
                            msg.header.frame_id, True)

                frame_transform = self.base_object.get_inverse_transform()
            if frame_transform is None:
                return

        new_header = Header(stamp=msg.header.stamp, frame_id=self.base_frame)

        for obj in self.objects:
            with obj.get_lock():
                obj.update_pose(transforms_dict, frame_transform)
                if not obj.is_ready():
                    continue

                marker = obj.generate_next_marker(new_header)
                if not marker:
                    continue

                obj.publish_transform(self.base_frame)
                new_msg.markers.append(marker)

        new_msg.markers.extend(self._get_test_aruco_markers(msg))
        self.pub.publish(new_msg)

    def _get_test_aruco_markers(self,
                                msg: FiducialTransformArray) -> 'list[Marker]':
        markers: list[Marker] = []
        transforms: list[FiducialTransform] = msg.transforms

        for t in transforms:
            id = t.fiducial_id
            size = self.sizes[id] if id in self.sizes else self.marker_size
            marker = create_basic_marker(msg.header)
            marker.header.frame_id = f'fiducial_{id}'
            marker.id = id
            marker.ns = 'aruco_tags'
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = 0.001
            marker.color.g = 1.0
            marker.lifetime = self.tags_lifetime
            markers.append(marker)

        return markers

    def _get_test_mesh_model(self, file: str, seq: int = 0) -> Marker:
        marker = create_basic_marker(Header(seq=seq, frame_id='base_link'))
        marker.type = Marker.MESH_RESOURCE
        marker.id = 1
        marker.ns = 'test'
        marker.color.g = 1.0
        marker.color.r = 1.0
        marker.color.b = 1.0
        marker.mesh_use_embedded_materials = True
        marker.mesh_resource = f'package://panel_visualization/mesh/{file}'

        return marker


class VisulObject:
    def __init__(self, parameters: dict, fileter_size: int,
                 lifetime: rospy.Duration) -> None:
        name: str = parameters['name']
        file: str = parameters['file']
        self.ros_interface = ROSInterface(name, file, lifetime)
        self.tags: list[Tag] = [
            Tag(tag['id'], tag['position'], tag['rotation'])
            for tag in parameters['tags']
        ]
        self.filter_size = fileter_size
        self.counter = 0
        self.last_pose: 'np.matrix | None' = None
        self.pose_buffer: 'list[tuple[np.matrix, float]]' = []
        self.duration_of_no_pose = 0
        self.mutex = Lock()

    def get_lock(self) -> Lock:
        return self.mutex

    def update_pose(self, transform_msgs: 'dict[int, FiducialTransform]',
                    camera_to_base: 'Transform | None'):
        for tag in self.tags:
            if tag.id not in transform_msgs:
                tag.update(None, 0.0)
                continue

            msg = transform_msgs[tag.id]
            t = msg.transform
            pose_matrix = self.ros_interface.matrix_from_ros_transform(t)
            tag.update(pose_matrix, msg.object_error)

        pose, weight = self._calculate_pose()

        if pose is not None:
            if camera_to_base is not None:
                t_c2b = self.ros_interface.matrix_from_ros_transform(
                    camera_to_base)
                based_pose = np.matmul(t_c2b, pose)
            else:
                based_pose = pose
            self.pose_buffer.append((based_pose, weight))

        self.counter += 1
        self.counter %= self.filter_size

    def is_ready(self) -> bool:
        return not self.counter

    def get_inverse_transform(self) -> 'Transform | None':
        if self.last_pose is None:
            return None

        camera_pose = np.linalg.inv(self.last_pose)
        return self.ros_interface.ros_transform_from_matrix(camera_pose)

    def publish_transform(self, frame_id: str, inverse=False) -> None:
        if self.last_pose is not None:
            self.ros_interface.publish_transform(frame_id, self.last_pose,
                                                 inverse)

    def generate_next_marker(self, header: Header) -> 'Marker | None':
        current_pose = self._get_average_pose()
        self.pose_buffer.clear()
        is_detected = current_pose is not None
        is_not_found_at_all = self.last_pose is None
        is_restored = is_detected and bool(self.duration_of_no_pose)

        if not is_detected and is_not_found_at_all:
            self.duration_of_no_pose += 1
            return None

        if is_detected:
            self.last_pose = current_pose
            self.duration_of_no_pose = 0
        else:
            self.duration_of_no_pose += 1

        color = self._get_color(is_detected, is_restored)
        marker = self.ros_interface.create_marker(header, color,
                                                  self.last_pose)

        return marker

    def _get_average_pose(self) -> 'np.matrix | None':
        if not len(self.pose_buffer):
            return None

        p_a = [0.0, 0.0, 0.0]
        q_a = [0.0, 0.0, 0.0, 1.0]

        current_weight = 0.0
        for pose, weight in self.pose_buffer:
            t = tft.translation_from_matrix(pose)
            q = tft.quaternion_from_matrix(pose)
            p_a += t * weight
            current_weight += weight
            q_a = tft.quaternion_slerp(q_a, q, weight / current_weight)

        p_a = p_a / current_weight
        t_a = tft.translation_matrix(p_a)
        r_a = tft.quaternion_matrix(q_a)
        pose_matrix = np.matmul(t_a, r_a)

        return pose_matrix

    # TODO: new rules for coloring in case of pose averaging
    def _get_color(self, is_detected: bool, is_restored: bool) -> ColorRGBA:
        color = ColorRGBA(a=1.0)
        valid_tags_num = sum(1 for tag in self.tags if tag.is_visible())

        if is_detected:
            if valid_tags_num < len(self.tags):
                color.r = 0.5
                color.g = 0.5
                color.b = 1.0
            elif is_restored:
                color.r = 0.5
                color.g = 1.0
                color.b = 0.5
            else:
                color.r = 1.0
                color.g = 1.0
                color.b = 1.0
        else:
            if self.duration_of_no_pose <= WARNING_DURATION:
                color.r = 1.0
                color.g = 1.0
            else:
                color.r = 1.0
                color.g = 0.5
                color.b = 0.5

        return color

    def _calculate_pose(self) -> 'tuple[np.matrix | None, float]':
        valid_tags = [tag for tag in self.tags if tag.is_visible()]

        if len(valid_tags) == 0:
            return None, 0.0
        elif len(valid_tags) == 1:
            return self._calculate_pose_from_single_tag(valid_tags[0])
        else:
            return self._calculate_pose_from_multi_tags(valid_tags)

    def _calculate_pose_from_single_tag(
            self, tag: 'Tag') -> 'tuple[np.matrix, float]':
        pose_matrix = tag.get_panel_pose_matrix()
        t_b2c = tag.get_transform_to_camera_rf()
        panel_pose = np.matmul(t_b2c, pose_matrix)
        weight = 1 / tag.get_uncertainty()

        return panel_pose, weight

    def _calculate_pose_from_multi_tags(
            self, valid_tags: 'list[Tag]') -> 'tuple[np.matrix, float]':
        weights = [1 / tag.get_uncertainty() for tag in valid_tags]
        p_poses = [
            tft.translation_from_matrix(tag.get_pose_matrix_from_panel_rf())
            for tag in valid_tags
        ]
        c_poses = [
            tft.translation_from_matrix(tag.get_pose_matrix_from_camera_rf())
            for tag in valid_tags
        ]
        mid_point_from_panel = np.average(p_poses, axis=0, weights=weights)
        mid_point_from_camera = np.average(c_poses, axis=0, weights=weights)
        mid_t_p = tft.translation_matrix(mid_point_from_panel)
        mid_t_c = tft.translation_matrix(mid_point_from_camera)

        t_m2c = mid_t_c
        t_c2m = np.linalg.inv(t_m2c)
        panel_orientations: 'list[tuple[np.matrix, float]]' = []

        for tag in valid_tags:
            t_p2b = tag.get_transform_from_panel_rf()
            t_c2b = tag.get_transform_from_camera_rf()
            t_p = tft.translation_from_matrix(np.matmul(t_p2b, mid_t_p))
            t_c = tft.translation_from_matrix(np.matmul(t_c2b, mid_t_c))
            axis = np.cross(t_p, t_c)
            t_p_norm = t_p / np.linalg.norm(t_p)
            t_c_norm = t_c / np.linalg.norm(t_c)
            angle = np.math.acos(np.dot(t_p_norm, t_c_norm))
            r_mat = tft.rotation_matrix(angle, axis)

            base_panel_pose = tag.get_panel_pose_matrix()
            corrected_pose = np.matmul(r_mat, base_panel_pose)
            t_b2c = tag.get_transform_to_camera_rf()
            t_b2m = np.matmul(t_c2m, t_b2c)
            pose_in_mid_rf = np.matmul(t_b2m, corrected_pose)
            orientation = tft.quaternion_from_matrix(pose_in_mid_rf)
            panel_orientations.append((orientation, 1 / tag.get_uncertainty()))

        current_orientation, current_weight = panel_orientations[0]
        for q, w in panel_orientations[1:]:
            current_weight += w
            current_orientation = tft.quaternion_slerp(current_orientation, q,
                                                       w / current_weight)

        panel_orientation = tft.quaternion_matrix(current_orientation)
        panel_position = np.linalg.inv(mid_t_p)  # in rf with panel orientation
        panel_pose = np.matmul(panel_orientation, panel_position)
        pose_in_camera_rf = np.matmul(t_m2c, panel_pose)

        return pose_in_camera_rf, sum(weights)


class ROSInterface:
    def __init__(self, name: str, file: str, period: rospy.Duration) -> None:
        self.name = name
        self.file = file
        self.marker_lifetime = period
        self.broadcaster = TransformBroadcaster()

    def ros_pose_from_matrix(self, pose_matrix: np.matrix) -> Pose:
        t: np.ndarray = tft.translation_from_matrix(pose_matrix)
        q: np.ndarray = tft.quaternion_from_matrix(pose_matrix)
        pose = Pose()
        pose.position.x = t[0]
        pose.position.y = t[1]
        pose.position.z = t[2]
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    def matrix_from_ros_pose(self, pose: Pose) -> np.matrix:
        t = pose.position
        q = pose.orientation
        translation = tft.translation_matrix([t.x, t.y, t.z])
        rotation = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        pose_matrix = np.matmul(translation, rotation)
        return pose_matrix

    def matrix_from_ros_transform(self, transform: Transform) -> np.matrix:
        t = transform.translation
        q = transform.rotation
        t_mat = tft.translation_matrix([t.x, t.y, t.z])
        r_mat = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        transform_matrix = np.matmul(t_mat, r_mat)
        return transform_matrix

    def ros_transform_from_matrix(self, t_matrix: np.matrix) -> Transform:
        t: np.ndarray = tft.translation_from_matrix(t_matrix)
        q: np.ndarray = tft.quaternion_from_matrix(t_matrix)
        transform = Transform()
        transform.translation.x = t[0]
        transform.translation.y = t[1]
        transform.translation.z = t[2]
        transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return transform

    def publish_transform(self,
                          frame_id: str,
                          t_matrix: np.matrix,
                          inverse=False) -> None:
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        if not inverse:
            msg.header.frame_id = frame_id
            msg.child_frame_id = self.name
            msg.transform = self.ros_transform_from_matrix(t_matrix)
        else:
            msg.header.frame_id = self.name
            msg.child_frame_id = frame_id
            inv_t_matrix = np.linalg.inv(t_matrix)
            msg.transform = self.ros_transform_from_matrix(inv_t_matrix)

        self.broadcaster.sendTransform(msg)

    def create_marker(self, header: Header, color: ColorRGBA,
                      pose_matrix: np.matrix) -> Marker:
        marker = create_basic_marker(header)
        marker.color = color
        marker.pose = self.ros_pose_from_matrix(pose_matrix)
        marker.id = 0
        marker.ns = self.name
        marker.lifetime = self.marker_lifetime
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = f'package://panel_visualization/mesh/{self.file}'
        marker.mesh_use_embedded_materials = True

        return marker


class Tag:
    def __init__(self, id, position: dict, orientation: dict) -> None:
        self.id = int(id)
        t_mat = tft.translation_matrix([val for val in position.values()])
        r_mat = tft.quaternion_matrix([val for val in orientation.values()])
        self.pose_from_panel: np.matrix = np.matmul(t_mat, r_mat)
        self.pose_from_camera: 'np.matrix | None' = None
        self.object_error = 0.0
        self.duration_of_no_transform = 0

    def update(self, pose_matrix: 'np.matrix | None', obj_err: float) -> None:
        if pose_matrix is not None:
            self.pose_from_camera = pose_matrix
            self.object_error = obj_err
            self.duration_of_no_transform = 0
        else:
            self.duration_of_no_transform += 1

    def is_visible(self) -> bool:
        return not self.duration_of_no_transform

    def get_uncertainty(self) -> float:
        return self.object_error

    def get_panel_pose_matrix(self) -> np.matrix:
        return np.linalg.inv(self.pose_from_panel)

    def get_pose_matrix_from_panel_rf(self) -> np.matrix:
        return np.copy(self.pose_from_panel)

    def get_transform_from_panel_rf(self) -> np.matrix:
        return np.linalg.inv(self.pose_from_panel)

    def get_pose_matrix_from_camera_rf(self) -> 'np.matrix | None':
        return np.copy(self.pose_from_camera)

    def get_transform_from_camera_rf(self) -> 'np.matrix | None':
        return np.linalg.inv(self.pose_from_camera)

    def get_transform_to_camera_rf(self) -> 'np.matrix | None':
        return np.copy(self.pose_from_camera)


def create_basic_marker(header: Header) -> Marker:
    marker = Marker()
    marker.header.seq = header.seq
    if header.stamp.secs or header.stamp.nsecs:
        marker.header.stamp = header.stamp
    else:
        marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = header.frame_id
    marker.action = marker.ADD
    marker.type = marker.CUBE
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.a = 1.0
    return marker


if __name__ == "__main__":
    try:
        PanelTracker().run()
    except:
        pass