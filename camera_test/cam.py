import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
import numpy as np
import math
from matplotlib import pyplot as plt


class CameraTest(Node):

    def __init__(self):
        super().__init__('cone_detector_node')
        self.frame = np.zeros((1, 1, 1), dtype=np.uint8)
        self.frame_tmp = np.zeros((1, 1, 1), dtype=np.uint8)
        self.bridge = CvBridge()
        self.drawing = False
        self.center_x = -1
        self.center_y = -1
        self.zoom_window_size = 50
        self.zoom_scale = 10.0
        self.zoomed_frame = np.zeros((1, 1, 1), dtype=np.uint8)
        self.x0 = 0
        self.x1 = 0
        self.y0 = 0
        self.y1 = 0
        self.all_colors = []
        
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth=1)
        #self.publisher = self.create_publisher(Image, '/cone_detection/viz', 10)
        self.subscription = self.create_subscription(Image, '/image', self.listener_callback, qos_profile)
        cv2.namedWindow('GUI')
        cv2.namedWindow('Zoom')
        cv2.setMouseCallback('GUI', self.mouse_callback)
        
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.center_x = x
            self.center_y = y
            self.frame_tmp = np.copy(self.frame)
        elif event == cv2.EVENT_LBUTTONUP:
            mask = np.zeros(self.frame.shape, dtype=np.uint8)
            radius = int(math.sqrt(math.pow(x - self.center_x, 2) + math.pow(y - self.center_y, 2)))
            mask = cv2.circle(mask, (self.center_x, self.center_y), radius, (1, 1, 1), -1)
            masked_frame = mask * self.frame_tmp
            #colors, counts = np.unique(masked_frame.reshape(-1, 3), axis=0, return_counts=True)
            self.all_colors = [*self.all_colors, *masked_frame.reshape(-1, 3).tolist()]
            self.drawing = False
        elif event == cv2.EVENT_MOUSEMOVE:  
            if self.drawing == False:
                self.x0 = max(0, x - self.zoom_window_size // 2)
                self.x1 = min(self.frame.shape[1] - 1, x + self.zoom_window_size // 2)
                self.y0 = max(0, y - self.zoom_window_size // 2)
                self.y1 = min(self.frame.shape[0] - 1, y + self.zoom_window_size // 2)
                self.zoomed_frame = self.frame[self.y0:self.y1, self.x0:self.x1, :]
                self.zoomed_frame = cv2.resize(self.zoomed_frame, None, fx=self.zoom_scale, fy=self.zoom_scale, interpolation=cv2.INTER_NEAREST)
                zfc_x = self.zoomed_frame.shape[0] // 2
                zfc_y = self.zoomed_frame.shape[1] // 2
                self.zoomed_frame = cv2.line(self.zoomed_frame, (zfc_x - 50, zfc_y), (zfc_x + 50, zfc_y), (0, 0, 255), 3)
                self.zoomed_frame = cv2.line(self.zoomed_frame, (zfc_x, zfc_y - 50), (zfc_x, zfc_y + 50), (0, 0, 255), 3)
            else:
                self.zoomed_frame = self.frame[self.y0:self.y1, self.x0:self.x1, :]
                self.zoomed_frame = cv2.resize(self.zoomed_frame, None, fx=self.zoom_scale, fy=self.zoom_scale, interpolation=cv2.INTER_NEAREST)
                self.frame = np.copy(self.frame_tmp)
                radius = int(math.sqrt(math.pow(x - self.center_x, 2) + math.pow(y - self.center_y, 2)))
                self.frame = cv2.circle(self.frame, (self.center_x, self.center_y), radius, (0, 255, 0), 1)


    def listener_callback(self, msg):
        t0 = time.time()
        if self.drawing == False:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("GUI", self.frame)
        cv2.imshow("Zoom", self.zoomed_frame)
        k = cv2.waitKey(1)
        if k == 27:
            self.all_colors = np.array(self.all_colors)
            print("all_colors:", self.all_colors.shape)

            colors, counts = np.unique(self.all_colors, axis=0, return_counts=True)
            b, g, r = colors[:, 0], colors[:, 1], colors[:, 2]
            b_min = b[b > 0].min()
            b_max = b[b > 0].max()
            g_min = g[g > 0].min()
            g_max = g[g > 0].max()
            r_min = r[r > 0].min()
            r_max = r[r > 0].max()
            print("b:", b_min, b_max)
            print("g:", g_min, g_max)
            print("r:", r_min, r_max)
            b = self.all_colors[:, 0]
            g = self.all_colors[:, 1]
            r = self.all_colors[:, 2]
            b_min = b[b > 0].min()
            b_max = b[b > 0].max()
            g_min = g[g > 0].min()
            g_max = g[g > 0].max()
            r_min = r[r > 0].min()
            r_max = r[r > 0].max()
            print("----------")
            print("b:", b_min, b_max)
            print("g:", g_min, g_max)
            print("r:", r_min, r_max)
            self.all_colors = self.all_colors[np.where((self.all_colors[:, 0]>0) * (self.all_colors[:, 1]>0) * (self.all_colors[:, 2]>0))]
            self.all_colors = np.expand_dims(self.all_colors, 0).astype(np.uint8)
            self.all_colors = cv2.cvtColor(self.all_colors, cv2.COLOR_BGR2HSV)
            print(self.all_colors[:, 0].min(), self.all_colors[:, 0].max())
            print(self.all_colors[:, 1].min(), self.all_colors[:, 1].max())
            print(self.all_colors[:, 2].min(), self.all_colors[:, 2].max())
            print(self.all_colors.shape)
            for i, col in enumerate(('b', 'g', 'r')):
                hist = cv2.calcHist([self.all_colors], [i], None, [256], [0, 256])
                plt.plot(hist, color = col)
                plt.xlim([0, 256])
            plt.show()
            print(hist.shape)
            cv2.destroyAllWindows()
            self.destroy_node()
            exit()
        t1 = time.time()
        print("Hz:", 1.0 / (t1 - t0))


def main(args=None):
    rclpy.init()
    cdn = CameraTest()
    rclpy.spin(cdn)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
