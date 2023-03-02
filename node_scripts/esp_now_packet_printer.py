#!/usr/bin/env python

import rospy
from esp_now_ros.utils import ESPNOWROSInterface


def callback(src_address, data):

  rospy.loginfo('Packet from {} : {}'.format(src_address, data))


if __name__ == '__main__':

  rospy.init_node('esp_now_packet_printer')
  interface = ESPNOWROSInterface(callback)
  rospy.spin()
