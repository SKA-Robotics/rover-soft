#!/usr/bin/python3
import rospy
import numpy as np
import tf.transformations as tft
from geometry_msgs.msg import Pose, Point, Quaternion, Transform
from visualization_msgs.msg import Marker, MarkerArray
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

CAMERA_FRAME = 'camera_frame'


class MarkerSender:
    def __init__(self) -> None:
        rospy.init_node("marker_corrector")

        objects: list[dict] = rospy.get_param("~visual_objects", {})
        self.objects = [VisulObject(dict_obj) for dict_obj in objects]

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
            marker = obj.get_marker()
            marker.header.seq = msg.header.seq
            marker.header.stamp = msg.header.stamp
            new_msg.markers.append(marker)

        new_msg.markers.extend(self.get_aruco_markers(msg))
        # new_msg.markers.append(self._get_test_mesh_model(msg))
        self.pub.publish(new_msg)

    def get_aruco_markers(self, msg: FiducialTransformArray) -> 'list[Marker]':
        markers: list[Marker] = []
        transforms: list[FiducialTransform] = msg.transforms

        for t in transforms:
            id = t.fiducial_id
            size = 0.03 if id == 12 or id == 27 else 0.05
            marker = Marker()
            marker.action = Marker.ADD
            marker.type = Marker.CUBE
            marker.header.seq = msg.header.seq
            marker.header.stamp = msg.header.stamp
            marker.header.frame_id = f'fiducial_{id}'
            marker.id = id
            marker.ns = 'aruco'
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = 0.001
            marker.color.g = 1.0
            marker.color.a = 1.0
            marker.pose.orientation.w = 1.0
            marker.lifetime = rospy.Duration(1)
            markers.append(marker)

        return markers

    def _get_test_mesh_model(self, msg: FiducialTransformArray) -> Marker:
        marker = Marker()
        marker.action = Marker.ADD
        marker.type = Marker.MESH_RESOURCE
        marker.header.seq = msg.header.seq
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = 'base_link'
        marker.id = 1
        marker.ns = 'test'
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.g = 1.0
        marker.color.r = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.mesh_use_embedded_materials = True
        marker.mesh_resource = 'package://panel_visualization/mesh/B2.dae'

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

    def update_transforms(self, transforms: 'dict[int, FiducialTransform]'):
        for tag in self.tags:
            msg = transforms.get(tag.id)
            tag.update(msg)

    def get_marker(self) -> Marker:
        valid_tags = [
            tag for tag in self.tags if not tag.no_transform_duration
        ]

        marker = Marker()
        marker.header.frame_id = CAMERA_FRAME
        marker.id = 0
        marker.ns = self.name
        marker.action = Marker.ADD
        marker.type = Marker.MESH_RESOURCE
        pose = self.get_pose(valid_tags)
        marker.pose = pose if pose else Pose(position=Point(z=1.0),
                                             orientation=Quaternion(w=1.0))
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        if len(valid_tags):
            marker.color.g = 1.0
            marker.color.b = 1.0

        marker.color.r = 1.0
        marker.color.a = 1.0
        marker.mesh_resource = f'package://panel_visualization/mesh/{self.file}'
        marker.mesh_use_embedded_materials = True
        return marker

    def get_pose(self, valid_tags: 'list[Tag]') -> Pose:
        if not len(valid_tags):
            return self.last_pose

        if len(valid_tags) < 2:
            base_tag = valid_tags[0]
            pose_matrix = base_tag.panel_pose_from_tag()
            t_b2c = base_tag.transform_from_tag_to_camera()
            panel_pose = np.matmul(t_b2c, pose_matrix)
            p = tft.translation_from_matrix(panel_pose)
            q = tft.quaternion_from_matrix(panel_pose)

            return self.ros_pose_from_matrix(p, q)

        # TODO: Add possibility of more than 2 tags interpolation
        if len(valid_tags) > 2:
            rospy.logwarn(
                f'Too many tags ({len(valid_tags)}) on one object. Expected 1 or 2.'
            )

        err_sum = sum(tag.object_error for tag in valid_tags)
        panel_poses = []
        for tag, base_tag in [(valid_tags[0], valid_tags[1]),
                              (valid_tags[1], valid_tags[0])]:

            t_p2b = base_tag.transform_from_panel_to_tag()
            t_c2b = base_tag.transform_from_camera_to_tag()
            delta_c = np.matmul(t_c2b, [*tag.position_from_camera(), 1])
            delta_p = np.matmul(t_p2b, [*tag.position_from_panel(), 1])
            t_c = delta_c[:-1]
            t_p = delta_p[:-1]
            axis = np.cross(t_p, t_c)
            angle = np.math.acos(
                np.dot(t_p / np.linalg.norm(t_p), t_c / np.linalg.norm(t_c)))
            r_mat = tft.rotation_matrix(angle, axis)
            base_panel_pose = base_tag.panel_pose_from_tag()
            # rospy.loginfo(base_panel_pose)
            corrected_pose = np.matmul(r_mat, base_panel_pose)
            t_b2c = np.linalg.inv(t_c2b)
            panel_pose = np.matmul(t_b2c, corrected_pose)
            panel_position = tft.translation_from_matrix(panel_pose)
            panel_orientation = tft.quaternion_from_matrix(panel_pose)
            obj_err = tag.object_error / err_sum
            panel_poses.append((panel_position, panel_orientation, obj_err))

        p0, q0, err0 = panel_poses[0]
        p1, q1, err1 = panel_poses[1]
        p = p0 * err1 + p1 * err0
        q = tft.quaternion_slerp(q0, q1, err0)
        # rospy.loginfo(f'Quaterion 0: {q0}')
        # rospy.loginfo(f'Quaterion 1: {q1}')
        # rospy.loginfo(f'Quaterion 2: {q}')

        return self.ros_pose_from_matrix(p, q)

    def ros_pose_from_matrix(self, t, q) -> Pose:
        pose = Pose()
        pose.position = Point(x=t[0], y=t[1], z=t[2])
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose


class Tag:
    def __init__(self, id, posision: dict, orientation: dict):
        self.id = int(id)
        self.p_from_panel = tft.translation_matrix(
            [coord for coord in posision.values()])
        self.o_from_panel = tft.quaternion_matrix(
            [coord for coord in orientation.values()])
        self.p_from_cam: 'np.matrix | None' = None
        self.o_from_cam: 'np.matrix | None' = None
        self.object_error = 0.0
        self.no_transform_duration = 0

    def update(self, msg: FiducialTransform) -> None:
        if msg:
            t = msg.transform.translation
            q = msg.transform.rotation
            self.p_from_cam = tft.translation_matrix([t.x, t.y, t.z])
            self.o_from_cam = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
            self.object_error = msg.object_error
            self.no_transform_duration = 0
        else:
            self.no_transform_duration += 1

    def panel_pose_from_tag(self):
        return np.linalg.inv(np.matmul(self.p_from_panel, self.o_from_panel))

    def transform_from_panel_to_tag(self):
        return np.linalg.inv(np.matmul(self.p_from_panel, self.o_from_panel))

    def position_from_panel(self):
        return tft.translation_from_matrix(self.p_from_panel)

    def orientation_from_panel(self):
        return tft.quaternion_from_matrix(self.o_from_panel)

    def transform_from_camera_to_tag(self):
        return np.linalg.inv(np.matmul(self.p_from_cam, self.o_from_cam))

    def transform_from_tag_to_camera(self):
        return np.matmul(self.p_from_cam, self.o_from_cam)

    def position_from_camera(self):
        return tft.translation_from_matrix(self.p_from_cam)

    def orientation_from_camera(self):
        return tft.quaternion_from_matrix(self.o_from_cam)


if __name__ == "__main__":
    try:
        MarkerSender().run()
    except:
        pass