#!/usr/bin/env python

import rospy

from esp_now_ros.smart_device_protocol_interface import SDPInterface
from esp_now_ros import DataFrame


def callback(src_address, frame):
    rospy.loginfo("Packet from {} : {}".format(src_address, frame))


if __name__ == "__main__":
    rospy.init_node("smart_device_protocol_packet_printer")
    interface = SDPInterface(callback_data=callback, callback_meta=callback)
    end = rospy.Time.now() + rospy.Duration(5.0)
    r = rospy.Rate(1)
    while not rospy.is_shutdown() and rospy.Time.now() < end:
        r.sleep()
        rospy.loginfo("Waiting")
    frame = DataFrame(
        "Message Board to write",
        ["Test", 10000, "Test Message"],
    )
    interface.send((120, 33, 132, 149, 125, 148), frame, num_trial=2)
    rospy.loginfo("Sent packet")

    rospy.spin()
