#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

class BallTracker:
    def __init__(self):
        rospy.init_node('ball_tracker')
        self.camera_topic = rospy.get_param('~camera_topic', '/usb_cam/image_raw')
        self.min_radius = rospy.get_param('~min_radius', 5)
        self.actual_diameter = rospy.get_param('~actual_diameter', 40)  # mm
        self.focal_length = rospy.get_param('~focal_length', 658.05)    # px
        self.smooth_factor = rospy.get_param('~smoothing', 0.9)
        self.center_threshold = rospy.get_param('~center_smooth_threshold', 3)
        self.show_gui = rospy.get_param('~show_gui', True)

        self.lower_orange = np.array([11, 124, 175])
        self.upper_orange = np.array([25, 255, 255])
        self.prev_x, self.prev_y, self.prev_r = 0, 0, 0

        self.bridge = CvBridge()
        self.position_pub = rospy.Publisher('/target_position', Float32MultiArray, queue_size=1)
        self.mask_pub = rospy.Publisher('/ball_tracker/debug_mask', Image, queue_size=1)
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback)

        if self.show_gui:
            cv2.namedWindow('Mask', cv2.WINDOW_NORMAL)
            cv2.namedWindow('Ball Tracker', cv2.WINDOW_NORMAL)
            self.init_hsv_sliders()

    def init_hsv_sliders(self):
        cv2.createTrackbar('H min', 'Mask', self.lower_orange[0], 179, lambda x: self.update_hsv())
        cv2.createTrackbar('S min', 'Mask', self.lower_orange[1], 255, lambda x: self.update_hsv())
        cv2.createTrackbar('V min', 'Mask', self.lower_orange[2], 255, lambda x: self.update_hsv())
        cv2.createTrackbar('H max', 'Mask', self.upper_orange[0], 179, lambda x: self.update_hsv())
        cv2.createTrackbar('S max', 'Mask', self.upper_orange[1], 255, lambda x: self.update_hsv())
        cv2.createTrackbar('V max', 'Mask', self.upper_orange[2], 255, lambda x: self.update_hsv())

    def update_hsv(self):
        self.lower_orange[0] = cv2.getTrackbarPos('H min', 'Mask')
        self.lower_orange[1] = cv2.getTrackbarPos('S min', 'Mask')
        self.lower_orange[2] = cv2.getTrackbarPos('V min', 'Mask')
        self.upper_orange[0] = cv2.getTrackbarPos('H max', 'Mask')
        self.upper_orange[1] = cv2.getTrackbarPos('S max', 'Mask')
        self.upper_orange[2] = cv2.getTrackbarPos('V max', 'Mask')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        x, y, radius = 0, 0, 0
        distance_cm = 0

        if contours:
            c = max(contours, key=cv2.contourArea)
            ((x1, y1), radius1) = cv2.minEnclosingCircle(c)

            dx = x1 - self.prev_x
            dy = y1 - self.prev_y
            if abs(dx) < self.center_threshold: x1 = self.prev_x
            if abs(dy) < self.center_threshold: y1 = self.prev_y

            x = self.smooth_factor * x1 + (1 - self.smooth_factor) * self.prev_x
            y = self.smooth_factor * y1 + (1 - self.smooth_factor) * self.prev_y
            radius = self.smooth_factor * radius1 + (1 - self.smooth_factor) * self.prev_r

            self.prev_x, self.prev_y, self.prev_r = x, y, radius

            if radius > self.min_radius:
                distance_cm = (self.actual_diameter * self.focal_length) / (2 * radius) / 10.0
                msg_out = Float32MultiArray()
                msg_out.data = [int(x), int(y), int(distance_cm)]
                self.position_pub.publish(msg_out)

        # 发布调试图像
        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))

        if self.show_gui:
            self.visualize(frame, int(x), int(y), int(radius), distance_cm, mask)

    def visualize(self, frame, x, y, r, d, mask):
        h, w = frame.shape[:2]
        cv2.line(frame, (w//2, 0), (w//2, h), (255, 0, 0), 1)
        cv2.line(frame, (0, h//2), (w, h//2), (255, 0, 0), 1)
        if r > self.min_radius:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)
            cv2.putText(frame, f"Dist: {d:.1f} cm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        else:
            cv2.putText(frame, "No valid ball", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
        cv2.imshow('Ball Tracker', frame)
        cv2.imshow('Mask', mask)
        cv2.waitKey(1)

    def run(self):
        rospy.on_shutdown(lambda: cv2.destroyAllWindows())
        rospy.spin()

if __name__ == '__main__':
    BallTracker().run()
