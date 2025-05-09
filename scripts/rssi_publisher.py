#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray

import silvus_driver.rssi_reader as rr

class RssiPublisher(rr.RssiReader):
    def __init__(self, ip="192.168.132.15", port=30000):
        rr.RssiReader.__init__(self, ip, port)
        self.pub = rospy.Publisher('rssi', Int32MultiArray, queue_size=10)
        self.msg = Int32MultiArray()
        self.msg.data = [0] * 5

    def run(self):
        self.socket.settimeout(5)
        while not rospy.is_shutdown():
            try:
                packet = self.receivePacket()
            except Exception:
                print('timeout')
                continue
            nodeId, rssi = rr.parsePacket(packet)
            if nodeId != '0':
                self.msg.data[0] = int(nodeId)
                self.msg.data[1:] = rssi
                self.pub.publish(self.msg)

if __name__ == "__main__":
    rospy.init_node('rssi_publisher')
    rp = RssiPublisher()
    rp.run()
