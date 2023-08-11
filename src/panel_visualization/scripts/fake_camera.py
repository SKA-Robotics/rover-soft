#!/usr/bin/python3
from os.path import dirname
import cv2 as cv
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CameraInfo, Image

SCRIPT_DIR = dirname(__file__)


class FakeCamera:
    def __init__(self) -> None:
        rospy.init_node("fake_camera")
        self.info_pub = rospy.Publisher("/fake_camera_info",
                                        CameraInfo,
                                        queue_size=10)
        self.img_pub = rospy.Publisher("/fake_camera_stream",
                                       Image,
                                       queue_size=10)
        self.bridge = CvBridge()
        self.frame_counter = 0
        self.duration = rospy.Duration(1)
        self.img_buff = [
            cv.imread(f'{SCRIPT_DIR}/../temp/{i}.jpg') for i in range(0, 10)
        ]
        msg = CameraInfo()
        time = rospy.get_time()
        msg.header.stamp.secs = int(time)
        msg.header.stamp.nsecs = int(1e9 * (time - int(time)))
        msg.height, msg.width, _ = self.img_buff[0].shape
        msg.distortion_model = 'plumb_bob'
        self.info_pub.publish(msg)

    def run(self):
        while rospy.is_shutdown:
            img = self.img_buff[self.frame_counter % 10]
            self.frame_counter += 1
            msg = self.bridge.cv2_to_imgmsg(img)
            msg.header.frame_id = 'manip_camera'

            self.img_pub.publish(msg)
            rospy.sleep(self.duration)

    def test(self):
        cv.imshow(f'image_{0}', cv.cvtColor(self.img_buff[0],
                                            cv.COLOR_BGR2GRAY))
        cv.waitKey(2000)
        cv.destroyAllWindows()


if __name__ == "__main__":
    try:
        FakeCamera().run()
    except Exception as e:
        rospy.loginfo(e)