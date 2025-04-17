#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray

from silvus_driver.tof_reader import ToFReader

class ToFPublisher(ToFReader):
    def __init__(self, radio_ip="192.168.0.136", topic="current_tof", rate_hz=10.0):
        super().__init__(radio_ip)
        self.pub = rospy.Publisher(topic, Int32MultiArray, queue_size=10)
        self.msg = Int32MultiArray()
        self.rate = rospy.Rate(rate_hz)

        rospy.loginfo(f"tof_publisher: polling {radio_ip} @ {rate_hz}Hz")
    
    def run(self):
        while not rospy.is_shutdown():
            try:
                data = self.fetch_tof()
                self.msg.data = data
                self.pub.publish(self.msg)
            except Exception as e:
                rospy.logwarn(f"ToF fetch failed: {e}")
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('tof_publisher')
    ip = rospy.get_param("~radio_ip", "192.168.0.136")
    hz = rospy.get_param("~rate", 10.0)
    tp = ToFPublisher(radio_ip=ip, rate_hz=hz)
    tp.run()
