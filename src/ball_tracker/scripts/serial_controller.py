#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import Float32MultiArray

class SerialController:
    def __init__(self):
        rospy.init_node('serial_controller')
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        self.DEAD_ZONE_X = rospy.get_param('~deadzone_x', 2)
        self.DEAD_ZONE_Y = rospy.get_param('~deadzone_y', 2)
        self.DEAD_ZONE_DIST = rospy.get_param('~deadzone_dist', 0)

        self.last_x = None
        self.last_y = None
        self.last_dist = None

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            rospy.loginfo("串口已打开")
        except serial.SerialException as e:
            rospy.logerr(f"串口打开失败: {e}")
            self.ser = None

        rospy.Subscriber('/target_position', Float32MultiArray, self.position_callback)
        rospy.on_shutdown(self.shutdown)

    def position_callback(self, msg):
        if len(msg.data) < 3:
            return
        x, y, dist = int(msg.data[0]), int(msg.data[1]), int(msg.data[2])

        if self.last_x is None:
            self.send_serial(x, y, dist)
        elif (abs(x - self.last_x) > self.DEAD_ZONE_X or
              abs(y - self.last_y) > self.DEAD_ZONE_Y or
              abs(dist - self.last_dist) > self.DEAD_ZONE_DIST):
            self.send_serial(x, y, dist)

        self.last_x, self.last_y, self.last_dist = x, y, dist

    def send_serial(self, x, y, dist):
        if self.ser and self.ser.is_open:
            try:
                data = f"#{x}${y}*{dist}\r\n"
                self.ser.write(data.encode('utf-8'))
                rospy.loginfo(f"发送串口: {data.strip()}")
            except Exception as e:
                rospy.logerr(f"串口发送失败: {e}")

    def shutdown(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            rospy.loginfo("串口已关闭")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    SerialController().run()
