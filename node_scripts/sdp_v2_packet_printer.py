#!/usr/bin/env python

import rospy
from smart_device_protocol.smart_device_protocol_interface import SDPInterface
from smart_device_protocol.utils import address_str_to_tuple, address_tuple_to_str


class Node:
    def __init__(self, target_address_str=None):
        self.target_address = (
            address_str_to_tuple(target_address_str)
            if target_address_str is not None
            else None
        )
        self.interface = SDPInterface(
            callback_data=self.callback, callback_meta=self.callback
        )

    def callback(self, src_address, frame):
        if self.target_address is not None:
            if src_address == self.target_address:
                print(
                    "{} Packet from {}".format(
                        type(frame), address_tuple_to_str(src_address)
                    )
                )
                print("{}".format(frame))
        else:
            print(
                "{} Packet from {}".format(
                    type(frame), address_tuple_to_str(src_address)
                )
            )
            print("{}".format(frame))


if __name__ == "__main__":
    rospy.init_node("smart_device_protocol_packet_printer")
    target_address = rospy.get_param("~target_address", None)
    node = Node(target_address_str=target_address)
    rospy.spin()
