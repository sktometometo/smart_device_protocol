#!/usr/bin/env python

import time
import unittest

import rospy
import rostest
from smart_device_protocol.smart_device_protocol_interface import UWBSDPInterface


class TestCase(unittest.TestCase):
    def callback_test(self, src_address, content):
        print("{}: {}".format(src_address, content))
        self.called = True

    def test_sdp_interface(self):
        self.called = False
        rospy.init_node("test_sdp_interface")
        sdp_interface = UWBSDPInterface()
        sdp_interface.register_interface_callback(
            ("Light status", "?"), self.callback_test
        )
        time.sleep(15)
        self.assertTrue(len(sdp_interface.device_interfaces) > 0)
        self.assertTrue(self.called)
        sdp_interface.unregister_interface_callback(("Light status", "?"))
        self.called = False
        time.sleep(15)
        self.assertFalse(self.called)
        self.assertTrue(
            sdp_interface.device_interfaces[(120, 33, 132, 168, 0, 196)]["distance"]
            is not None
        )


if __name__ == "__main__":
    rostest.rosrun("smart_device_protocol", "test_sdp_interface", TestCase)
