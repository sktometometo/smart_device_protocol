#!/usr/bin/env python

import rospy

from esp_now_ros.sdp_interface import SDPInterfaceNode


if __name__ == "__main__":
    rospy.init_node("smart_device_interface")
    node = SDPInterfaceNode()
    node.run()
