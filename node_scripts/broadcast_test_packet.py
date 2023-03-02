#!/usr/bin/env python

import rospy
from esp_now_ros.utils import create_test_packet, ESPNOWROSInterface

if __name__ == '__main__':

  rospy.init_node('broadcast_test_packet')

  interface = ESPNOWROSInterface()

  rate = rospy.Rate(5)
  while not rospy.is_shutdown():
    rate.sleep()
    data = create_test_packet()
    interface.send((255, 255, 255, 255, 255, 255), data)
    rospy.info('send data: {}'.format(data))
