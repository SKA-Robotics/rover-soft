#!/usr/bin/python3
import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

INFO_TOPIC = "/fake_camera_info"
STREAM_TOPIC = "/fake_camera_stream"
MSG_RATE = 0.3  # Hz


class ImageReceiver:
    def __init__(self) -> None:
        rospy.init_node("image_receiver")
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
        ms_to_wait = int(1000 / MSG_RATE / 2)
        cv.waitKey(ms_to_wait)
        cv.destroyAllWindows()


if __name__ == "__main__":
    try:
        ImageReceiver().run()
    except:
        pass