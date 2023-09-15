#!/usr/bin/python3
from os.path import dirname
import cv2 as cv
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

SCRIPT_DIR = dirname(__file__)
GET_VIDEO = False


class FakeCamera:

    def __init__(self) -> None:
        rospy.init_node("fake_camera")

        MSG_RATE = float(rospy.get_param("~message_rate", "1.0"))  # Hz
        self.rate = rospy.Rate(MSG_RATE)

        self.frame_id = rospy.get_param("~frame_id", "manip_camera")
        self.seq_counter = 0

        NS = rospy.get_param("~camera_name", "/fake_camera")
        self.info_pub = rospy.Publisher(f'{NS}/camera_info', CameraInfo, queue_size=10)
        self.img_pub = rospy.Publisher(f'{NS}/image', Image, queue_size=10)

        self.bridge = CvBridge()

        ## Choose images for testing
        self.path = f'{SCRIPT_DIR}/../temp'
        self.img_buff = [cv.imread(f'{self.path}/test_{i}.jpg') for i in range(5, 6)]
        self.video = cv.VideoCapture(f'{self.path}/test_video.mp4')

    def __del__(self) -> None:
        self.video.release()
        cv.destroyAllWindows()

    def run(self) -> None:
        while not rospy.is_shutdown():
            if GET_VIDEO and self.video.isOpened():
                ok, img = self.video.read()
                if not ok:
                    self.video.release()
                    self.video.open(f'{self.path}/test_video.mp4')
                    if self.video.isOpened():
                        ok, img = self.video.read()
                    rospy.loginfo('Repeating video')
            else:
                img = self.img_buff[self.seq_counter % len(self.img_buff)]

            self._send_info(img)
            self._send_image(img)
            self.seq_counter += 1
            self.rate.sleep()

    def _send_info(self, img: cv.Mat) -> None:
        msg = CameraInfo()
        msg.header = self._get_header()
        msg.height, msg.width, _ = img.shape
        msg.distortion_model = 'plumb_bob'
        cx = msg.width / 2.0
        cy = msg.height / 2.0
        fx = 0.5 * (msg.width + msg.height)
        fy = fx
        msg.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(msg)

    def _send_image(self, img: cv.Mat) -> None:
        rgb_img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        msg = self.bridge.cv2_to_imgmsg(rgb_img, "rgb8", self._get_header())
        self.img_pub.publish(msg)

    def _get_header(self) -> Header:
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id
        header.seq = self.seq_counter
        return header

    def _test(self):
        cv.imshow(f'image_{0}', cv.cvtColor(self.img_buff[0], cv.COLOR_BGR2GRAY))
        cv.waitKey(2000)
        cv.destroyAllWindows()


if __name__ == "__main__":
    try:
        FakeCamera().run()
    except:
        pass
        # rospy.loginfo(e)
