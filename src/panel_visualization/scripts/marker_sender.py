#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray


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
            # TODO: Make better algorithm averaging multi tags pose
            obj.set_transforms(transforms_dict)
            marker = obj.get_marker()
            marker.header.seq = msg.header.seq
            marker.header.stamp = msg.header.stamp
            new_msg.markers.append(marker)

        self.pub.publish(new_msg)


class VisulObject:
    def __init__(self, data: dict) -> None:
        self.name: str = data['name']
        self.file: str = data['file']
        self.tags: list[Tag] = [
            Tag(tag['id'], tag['position'], tag['rotation'])
            for tag in data['tags']
        ]

    def set_transforms(self, transforms: 'dict[int, FiducialTransform]'):
        self.transforms = [transforms.get(tag.id) for tag in self.tags]

    def get_marker(self):
        id = self.tags[0].id
        marker = Marker()
        marker.header.frame_id = f'fiducial_{id}'
        marker.id = id
        marker.ns = self.name
        marker.action = Marker.ADD
        marker.type = Marker.MESH_RESOURCE
        marker.pose = self.get_pose()
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.mesh_resource = f'package://panel_visualization/mesh/{self.file}'
        marker.mesh_use_embedded_materials = True
        return marker

    def get_pose(self) -> Pose:
        p = self.tags[0].posision
        r = self.tags[0].orientation
        pose = Pose()
        pose.position = Point(x=-p.x, y=-p.y, z=-p.z)
        pose.orientation = Quaternion(x=-r.x, y=-r.y, z=-r.z, w=r.w)
        return pose


class Tag:
    def __init__(self, id, posision: dict, orientation: dict):
        self.id = int(id)
        self.posision = Point(*[coord for coord in posision.values()])
        self.orientation = Quaternion(
            *[coord for coord in orientation.values()])


if __name__ == "__main__":
    try:
        MarkerSender().run()
    except:
        pass