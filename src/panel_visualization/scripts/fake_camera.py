#!/usr/bin/python3
from os.path import dirname
import cv2 as cv
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

SCRIPT_DIR = dirname(__file__)

INFO_TOPIC = "/fake_camera_info"
STREAM_TOPIC = "/fake_camera_stream"
FRAME_ID = "/manip_camera"
MSG_RATE = 0.3  # Hz


class FakeCamera:
    def __init__(self) -> None:
        rospy.init_node("fake_camera")
        self.info_pub = rospy.Publisher(INFO_TOPIC, CameraInfo, queue_size=10)
        self.img_pub = rospy.Publisher(STREAM_TOPIC, Image, queue_size=10)
        self.bridge = CvBridge()
        self.seq_counter = 0
        self.rate = rospy.Rate(MSG_RATE)
        # self.img_buff = [
        #     cv.imread(f'{SCRIPT_DIR}/../temp/stream{i}.jpg') for i in range(0, 10)
        # ]
        self.img_buff = [
            cv.imread(f'{SCRIPT_DIR}/../temp/sample{i}.png') for i in range(0, 6)
        ]

    def __del__(self) -> None:
        cv.destroyAllWindows()

    def run(self) -> None:
        while rospy.is_shutdown:
            self._send_info()
            self._send_image()
            self.seq_counter += 1
            self.rate.sleep()

    def _send_info(self) -> None:
        msg = CameraInfo()
        msg.header = self._get_header()
        msg.height, msg.width, _ = self.img_buff[0].shape
        msg.distortion_model = 'plumb_bob'
        cx = msg.width / 2.0
        cy = msg.height / 2.0
        fx = 5.0 * (msg.width + msg.height)
        fy = fx
        msg.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(msg)

    def _send_image(self) -> None:
        img = self.img_buff[self.seq_counter % len(self.img_buff)]
        msg = self.bridge.cv2_to_imgmsg(img, "rgb8", self._get_header())
        self.img_pub.publish(msg)

    def _get_header(self) -> Header:
        header = Header()
        time = rospy.get_time()
        header.stamp.secs = int(time)
        header.stamp.nsecs = int(1e9 * (time - int(time)))
        header.frame_id = FRAME_ID
        header.seq = self.seq_counter
        return header

    def _test(self):
        cv.imshow(f'image_{0}', cv.cvtColor(self.img_buff[0],
                                            cv.COLOR_BGR2GRAY))
        cv.waitKey(2000)
        cv.destroyAllWindows()


if __name__ == "__main__":
    try:
        FakeCamera().run()
    except:
        pass
        # rospy.loginfo(e)
