#!/usr/bin/env python

import rospy

from esp_now_ros.esp_now_ros_interface import ESPNOWROSInterfaceNode


if __name__ == "__main__":
    rospy.init_node("smart_device_interface")
    node = ESPNOWROSInterfaceNode()
    node.run()
