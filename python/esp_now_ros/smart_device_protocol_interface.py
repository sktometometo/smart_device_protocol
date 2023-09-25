import struct
from esp_now_ros import MetaFrame, DataFrame
from esp_now_ros.esp_now_ros_interface import ESPNOWROSInterface
from esp_now_ros.packet_parser import parse_packet_as_v2
from esp_now_ros.msg import Packet


class SDPInterface:
    def __init__(self, callback_data=None, callback_meta=None):
        self.callback_data = callback_data
        self.callback_meta = callback_meta
        self.esp_now_ros_interface = ESPNOWROSInterface(self.callback)

    def callback(self, src_address, data):
        try:
            _, frame = parse_packet_as_v2(
                Packet(data=data, mac_address=struct.pack("6B", *list(src_address)))
            )
        except ValueError:
            return
        if isinstance(frame, DataFrame):
            self.callback_data(src_address, frame)
        elif isinstance(frame, MetaFrame):
            self.callback_meta(src_address, frame)

    def send(self, target_address, frame, num_trial=1):
        if isinstance(frame, DataFrame):
            data = frame.to_bytes()
        elif isinstance(frame, MetaFrame):
            data = frame.to_bytes()
        else:
            raise ValueError(f"Unknown frame type: {type(frame)}")
        self.esp_now_ros_interface.send(target_address, data, num_trial)
