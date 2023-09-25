#!/usr/bin/env python

import rospy

from esp_now_ros.smart_device_protocol_interface import SDPInterface


def callback(src_address, frame):
    rospy.loginfo("Packet from {} : {}".format(src_address, frame))


if __name__ == "__main__":
    rospy.init_node("smart_device_protocol_packet_printer")
    interface = SDPInterface(callback_data=callback, callback_meta=callback)
    rospy.spin()
