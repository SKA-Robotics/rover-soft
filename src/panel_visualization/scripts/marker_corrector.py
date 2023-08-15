#!/usr/bin/python3
import rospy
from visualization_msgs.msg import Marker

BASE_SIZE = 0.14  # [m]
SIGMA_OFFSET = 0.055
TEXT_OFFSET = 0.105


class MarkerCorrector:
    def __init__(self) -> None:
        rospy.init_node("marker_corrector")
        self.marker_size = float(rospy.get_param("~marker_length", "0.05"))
        lengths: str = rospy.get_param("~marker_lengths", "")
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

        rospy.Subscriber("/unscaled/fiducials",
                         Marker,
                         self._callback,
                         queue_size=10)
        self.pub = rospy.Publisher("/fiducials", Marker, queue_size=10)

    def run(self) -> None:
        rospy.spin()

    def _callback(self, msg: Marker) -> None:
        r = msg.pose.orientation
        if r.x == 0.0 and r.y == 0.0 and r.z == 0.0 and r.w == 0.0:
            r.w = 1.0

        id = msg.id % 10000
        size = self.sizes[id] if id in self.sizes else self.marker_size
        msg.scale.x *= size / BASE_SIZE
        msg.scale.y *= size / BASE_SIZE
        msg.scale.z *= size / BASE_SIZE

        if msg.ns == 'sigma':
            msg.pose.position.z += (size / BASE_SIZE - 1) * SIGMA_OFFSET
        elif msg.ns == 'text':
            msg.pose.position.z += (size / BASE_SIZE - 1) * TEXT_OFFSET

        self.pub.publish(msg)


if __name__ == "__main__":
    try:
        MarkerCorrector().run()
    except:
        pass