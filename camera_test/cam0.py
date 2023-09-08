import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
import numpy as np


class CameraTest(Node):

    def __init__(self):
        super().__init__('cone_detector_node')
        self.bridge = CvBridge()
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth=1)
        #self.publisher = self.create_publisher(Image, '/cone_detection/viz', 10)
        self.subscription = self.create_subscription(Image, '/image', self.listener_callback, qos_profile)
        cv2.namedWindow('GUI')
        cv2.setMouseCallback('GUI', self.mouse_callback)
        
    def mouse_callback(self, event, x, y, flags, param):
        print("EVENT:", event)


    def listener_callback(self, msg):
        t0 = time.time()
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #frame = np.array(frame, dtype=np.float32)
        #print(np.unique(frame))
        cv2.imshow("GUI", frame)
        cv2.waitKey(3)
        #self.publisher.publish(self.bridge.cv2_to_imgmsg(np.array(frame), "bgr8"))
        t1 = time.time()
        print("Hz:", 1.0 / (t1 - t0))


def main(args=None):
    rclpy.init()
    cdn = CameraTest()
    rclpy.spin(cdn)
    cdn.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
