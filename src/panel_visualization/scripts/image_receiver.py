#!/usr/bin/python3
import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ImageReceiver:
    def __init__(self) -> None:
        rospy.init_node("image_receiver")

        MSG_RATE = float(rospy.get_param("~message_rate", "1.0"))  # Hz
        self.ms_to_wait = int(1000 / MSG_RATE / 2)

        STREAM_TOPIC = rospy.get_param("~stream_topic", "/fake_camera_stream")
        self.img_sub = rospy.Subscriber(STREAM_TOPIC, Image,
                                        self._test_img_view)
        self.bridge = CvBridge()

    def __del__(self) -> None:
        cv.destroyAllWindows()

    def run(self) -> None:
        rospy.spin()

    def _test_img_view(self, msg: Image) -> None:
        img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        cv.imshow('test', img)
        cv.waitKey(self.ms_to_wait)
        cv.destroyAllWindows()


if __name__ == "__main__":
    try:
        ImageReceiver().run()
    except:
        pass