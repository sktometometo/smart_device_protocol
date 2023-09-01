import importlib
import struct
from typing import Callable, Dict, List, Optional, Tuple

import rospy

from esp_now_ros import DataFrame, MetaFrame, parse_packet
from esp_now_ros.msg import Packet


def import_ros_type(ros_type_str: str) -> type:
    package_name, message_name = ros_type_str.split("/")
    module = importlib.import_module(f"{package_name}.msg")
    rostype = getattr(module, message_name)
    return rostype


def address_tuple_to_str(address: Tuple) -> str:
    return "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}".format(
        address[0],
        address[1],
        address[2],
        address[3],
        address[4],
        address[5],
    )


def address_str_to_tuple(address: str) -> Tuple:
    return tuple([int(x, 16) for x in address.split(":")])


class ESPNOWROSInterfaceNode:
    def __init__(self):
        self.sub = rospy.Subscriber("/esp_now_ros/recv", Packet, self.callback)
        self.pub = rospy.Publisher("/esp_now_ros/send", Packet, queue_size=1)

        self.routing_config: Dict[
            Tuple[str, str], Tuple[str, type, Callable, Callable]
        ] = {}
        # Key:
        #   Tuple:
        #     - packet_description
        #     - format_specifier
        # Value:
        #   Tuple:
        #     - sub_topic_name
        #     - ros_type
        #     - input_conversion_function
        #     - output_conversion_function

        self.input_routing_table: Dict[
            Tuple[str, str, str],
            List[rospy.Publisher, type, Callable, rospy.Time, str, str],
        ] = {}
        # routing table from smart device protocol to ROS
        # Update based on DataFrame information
        # Key:
        #   Tuple
        #     - address (as str)
        #     - interface_description
        #     - format specifier
        # Value:
        #   List:
        #     - publisher
        #     - rostype
        #     - input conversion function
        #     - last used stamp
        #     - device_name
        #     - target topic name

        self.output_routing_table: Dict[
            str, List[rospy.Subscriber, type, Callable, rospy.Time, str, str, str]
        ] = {}
        # routing table from ROS to smart device protol
        # Updated based on MetaFrame information
        # Key: rostopic = /<device_name>/<sub_topic_name>
        # Values:
        #   List:
        #     - subscriber
        #     - rostype
        #     - output conversion function
        #     - last updated stamp
        #     - target address
        #     - interface_description
        #     - format specifier

    def callback(self, msg: Packet):
        src_address, frame = parse_packet(msg)

        # Update routing table
        if isinstance(frame, MetaFrame):
            device_name = frame.device_name
            mac_address = address_tuple_to_str(src_address)
            for (
                packet_description,
                serialization_format,
            ) in frame.interface_descriptions:
                key = (mac_address, packet_description, serialization_format)
                if key not in self.input_routing_table.keys():
                    self.input_routing_table[key][3] = rospy.Time.now()
                else:
                    (
                        ros_sub_topic,
                        ros_msg_type,
                        input_conversion_function,
                        output_conversion_function,
                    ) = self.routing_config[(packet_description, serialization_format)]
                    ros_topic = f"/{device_name}/{ros_sub_topic}"
                    self.input_routing_table[key] = [
                        rospy.Publisher(ros_topic, ros_msg_type, queue_size=1),
                        ros_msg_type,
                        input_conversion_function,
                        rospy.Time.now(),
                        device_name,
                        ros_topic,
                    ]
        # 上下逆!!!!
        elif isinstance(frame, DataFrame):
            # Get device name from routing tables
            # TODO
            device_name = "test"
            mac_address = address_tuple_to_str(src_address)
            packet_description = frame.packet_description
            serialization_format = frame.serialization_format
            (
                ros_sub_topic,
                ros_msg_type,
                input_conversion_function,
                output_conversion_function,
            ) = self.routing_config[(packet_description, serialization_format)]
            ros_topic = f"/{device_name}/{ros_sub_topic}"
            if ros_topic in self.output_routing_table.keys():
                self.output_routing_table[ros_topic][3] = rospy.Time.now()
            else:
                self.output_routing_table[ros_topic] = [
                    rospy.Subscriber(ros_topic, ros_msg_type, lambda m: print(m)),
                    ros_msg_type,
                    output_conversion_function,
                    rospy.Time.now(),
                    mac_address,
                ]

        # Packet conversion and publish
        if isinstance(frame, DataFrame):
            print("hoge")

    def load_routing_config(self, config: List[Dict]):
        for entry in config:
            try:
                sdp_description = entry["smart_device_protocol"]["description"]
                sdp_format_specifier = entry["smart_device_protocol"][
                    "format_specifier"
                ]
                ros_subtopic: str = entry["ros"]["subtopic"]
                ros_msg_type: type = import_ros_type(entry["ros"]["msg_type"])
                input_conversion_function: Callable = eval(
                    "lambda d: {}".format(entry["input_conversion_function"])
                )
                output_conversion_function: Callable = eval(
                    "lambda d: {}".format(entry["output_conversion_function"])
                )
                self.routing_config[(sdp_description, sdp_format_specifier)] = (
                    ros_subtopic,
                    ros_msg_type,
                    input_conversion_function,
                    output_conversion_function,
                )
            except (ValueError, TypeError, SyntaxError):
                rospy.logerr("Failed to load a entry: {}".format(entry))

    def send(self, target_address: List[int], data: bytes, num_trial=1):
        """
        Args:
            target_address (list of int)
            data (bytes)
        """
        msg = Packet()
        msg.mac_address = struct.pack(
            "6B",
            target_address[0],
            target_address[1],
            target_address[2],
            target_address[3],
            target_address[4],
            target_address[5],
        )
        msg.data = data
        for _ in range(num_trial):
            self.pub.publish(msg)
