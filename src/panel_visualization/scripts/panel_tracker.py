#!/usr/bin/python3
import numpy as np
import tf.transformations as tft

import rospy
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

WARNING_DURATION = 3  # [numer of frames]


class PanelTracker:
    def __init__(self) -> None:
        rospy.init_node("panel_tracker")

        message_rate: float = rospy.get_param("~message_rate", 1.0)
        secs = int(1.0 / message_rate)
        nsecs = int(1e9 * (1.0 / message_rate - secs))
        self.marker_lifetime = rospy.Duration(secs, nsecs)

        objects: list[dict] = rospy.get_param("~visual_objects", {})
        self.objects = [VisulObject(dict_obj) for dict_obj in objects]

        self.marker_size: float = rospy.get_param("~marker_length", 0.05)
        lengths: str = rospy.get_param("~marker_lengths_override", "")
        self.sizes: dict[int, float] = {}
        try:
            for (k, v) in [scope.split(':') for scope in lengths.split(',')]:
                if k.find('-') != -1:
                    for (s, e) in [k.split('-')]:
                        self.sizes.update(
                            {i: float(v)
                             for i in range(int(s),
                                            int(e) + 1)})
                else:
                    self.sizes.update({int(k): float(v)})
        except:
            pass

        rospy.Subscriber("/fiducial_transforms",
                         FiducialTransformArray,
                         self._callback,
                         queue_size=10)
        self.pub = rospy.Publisher("/visual_objects",
                                   MarkerArray,
                                   queue_size=10)

    def run(self) -> None:
        rospy.spin()

    def _callback(self, msg: FiducialTransformArray) -> None:
        new_msg = MarkerArray()
        transforms: list[FiducialTransform] = msg.transforms
        transforms_dict = {t.fiducial_id: t for t in transforms}

        for obj in self.objects:
            obj.update_transforms(transforms_dict)
            marker = obj.get_marker(msg.header)
            if marker:
                # marker.header.frame_id = marker.ns
                # marker.pose = Pose()
                # marker.pose.orientation.w = 1.0
                obj.publish_transform(msg.header.frame_id)
                marker.lifetime = self.marker_lifetime
                new_msg.markers.append(marker)

        new_msg.markers.extend(self._get_aruco_markers(msg))
        # new_msg.markers.append(self._get_test_mesh_model('B2.dae', msg.header.seq))
        self.pub.publish(new_msg)

    def _get_aruco_markers(self,
                           msg: FiducialTransformArray) -> 'list[Marker]':
        markers: list[Marker] = []
        transforms: list[FiducialTransform] = msg.transforms

        for t in transforms:
            id = t.fiducial_id
            size = self.sizes[id] if id in self.sizes else self.marker_size
            marker = create_basic_marker(msg.header)
            marker.header.frame_id = f'fiducial_{id}'
            marker.id = id
            marker.ns = 'aruco'
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = 0.001
            marker.color.g = 1.0
            marker.lifetime = self.marker_lifetime
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
    def __init__(self, data: dict) -> None:
        self.name: str = data['name']
        self.file: str = data['file']
        self.tags: list[Tag] = [
            Tag(tag['id'], tag['position'], tag['rotation'])
            for tag in data['tags']
        ]
        self.last_pose: 'Pose | None' = None
        self.duration_of_no_pose = 0
        self.broadcaster = TransformBroadcaster()

    def update_transforms(self, transforms: 'dict[int, FiducialTransform]'):
        for tag in self.tags:
            msg = transforms.get(tag.id)
            tag.update(msg)

    def publish_transform(self, frame_id: str) -> None:
        msg = TransformStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = rospy.Time.now()
        msg.child_frame_id = self.name
        pose = self.last_pose
        if not pose:
            return
        msg.transform.translation.x = pose.position.x
        msg.transform.translation.y = pose.position.y
        msg.transform.translation.z = pose.position.z
        msg.transform.rotation = pose.orientation
        self.broadcaster.sendTransform(msg)

    def get_marker(self, header: Header) -> 'Marker | None':
        valid_tags = [tag for tag in self.tags if tag.is_visible()]

        marker = create_basic_marker(header)
        current_pose = self._get_pose(valid_tags)

        if not current_pose and not self.last_pose:
            self.duration_of_no_pose += 1
            return None

        marker.color = self._get_color(bool(current_pose), len(valid_tags))

        self.duration_of_no_pose = 0 if current_pose else self.duration_of_no_pose + 1
        marker.pose = current_pose if current_pose else self.last_pose
        self.last_pose = marker.pose

        marker.id = 0
        marker.ns = self.name
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = f'package://panel_visualization/mesh/{self.file}'
        marker.mesh_use_embedded_materials = True

        return marker

    def _get_pose(self, valid_tags: 'list[Tag]') -> 'Pose | None':
        if not len(valid_tags):
            return None

        if len(valid_tags) < 2:
            return self._calculate_pose_from_single_tag(valid_tags[0])

        return self._calculate_pose_from_multi_tags(valid_tags)

    def _get_color(self, is_detected: bool, valid_tags_num: int) -> ColorRGBA:
        color = ColorRGBA(a=1.0)

        if is_detected:
            if valid_tags_num < len(self.tags):
                color.r = 0.5
                color.g = 0.5
                color.b = 1.0
            elif self.duration_of_no_pose == 0:
                color.r = 1.0
                color.g = 1.0
                color.b = 1.0
            else:
                color.r = 0.5
                color.g = 1.0
                color.b = 0.5
        else:
            if self.duration_of_no_pose < WARNING_DURATION:
                color.r = 1.0
                color.g = 1.0
            else:
                color.r = 1.0
                color.g = 0.5
                color.b = 0.5

        return color

    def _calculate_pose_from_single_tag(self, base_tag: 'Tag') -> Pose:
        pose_matrix = base_tag.get_panel_pose_matrix()
        t_b2c = base_tag.get_transform_to_camera_rf()
        panel_pose = np.matmul(t_b2c, pose_matrix)

        return self._ros_pose_from_matrix(panel_pose)

    def _calculate_pose_from_multi_tags(self, valid_tags: 'list[Tag]') -> Pose:
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

        return self._ros_pose_from_matrix(pose_in_camera_rf)

    def _ros_pose_from_matrix(self, panel_pose: np.matrix) -> Pose:
        t = tft.translation_from_matrix(panel_pose)
        q = tft.quaternion_from_matrix(panel_pose)
        pose = Pose()
        pose.position = Point(x=t[0], y=t[1], z=t[2])
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose


class Tag:
    def __init__(self, id, posision: dict, orientation: dict) -> None:
        self.id = int(id)
        t_mat = tft.translation_matrix([val for val in posision.values()])
        r_mat = tft.quaternion_matrix([val for val in orientation.values()])
        self.pose_from_panel = np.matmul(t_mat, r_mat)
        self.pose_from_camera: 'np.matrix | None' = None
        self.object_error = 0.0
        self.duration_of_no_transform = 0

    def update(self, msg: FiducialTransform) -> None:
        if msg:
            t = msg.transform.translation
            q = msg.transform.rotation
            t_mat = tft.translation_matrix([t.x, t.y, t.z])
            r_mat = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
            self.pose_from_camera = np.matmul(t_mat, r_mat)
            self.object_error = msg.object_error
            self.duration_of_no_transform = 0
        else:
            self.duration_of_no_transform += 1

    def is_visible(self) -> bool:
        return not self.duration_of_no_transform

    def get_uncertainty(self) -> float:
        return self.object_error

    def get_panel_pose_matrix(self):
        return np.linalg.inv(self.pose_from_panel)

    def get_pose_matrix_from_panel_rf(self):
        return self.pose_from_panel

    def get_transform_from_panel_rf(self):
        return np.linalg.inv(self.pose_from_panel)

    def get_pose_matrix_from_camera_rf(self):
        return self.pose_from_camera

    def get_transform_from_camera_rf(self):
        return np.linalg.inv(self.pose_from_camera)

    def get_transform_to_camera_rf(self):
        return self.pose_from_camera


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