#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray

import numpy as np

from silvus_driver.rssi_reader import RssiReader

class RssiPublisher(RssiReader):
    def __init__(self, ip="192.168.132.15", port=30000):
        RssiReader.__init__(self, ip, port)
        self.pub = rospy.Publisher('rssi', Int32MultiArray, queue_size=10)
        self.msg = Int32MultiArray()
        self.msg.data = np.zeros(5)

    def run(self):
        self.socket.settimeout(5)
        while not rospy.is_shutdown():
            try:
                packet = self.receivePacket()
            except Exception:
                print('timeout')
                continue
            nodeId, rssi = parsePacket()
            if nodeId != '0':
                self.msg.data[0] = int(nodeId)
                self.msg.data[1:] = rssi
                self.pub.publish(self.msg)

if __name__ == "__main__":
    rospy.init_node('rssi_publisher')
    rp = RssiPublisher()
    rp.run()
