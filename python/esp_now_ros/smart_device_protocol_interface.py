import struct
from typing import Callable, Union, List, Tuple, Optional

from esp_now_ros import MetaFrame, DataFrame, BaseFrame
from esp_now_ros.esp_now_ros_interface import ESPNOWROSInterface
from esp_now_ros.packet_parser import parse_packet_as_v2
from esp_now_ros.msg import Packet


class SDPInterface:
    """Smart Device Protocol Interface"""

    def __init__(
        self,
        callback_data: Optional[
            Callable[[Union[List[int], Tuple[int]], BaseFrame], None]
        ] = None,
        callback_meta: Optional[
            Callable[[Union[List[int], Tuple[int]], BaseFrame], None]
        ] = None,
    ):
        """Smart Device Protocol Interface

        Args:
            callback_data (Optional[ Callable[[Union[List[int], Tuple[int]], BaseFrame], None] ], optional): callback function for DataFrame. Defaults to None.
            callback_meta (Optional[ Callable[[Union[List[int], Tuple[int]], BaseFrame], None] ], optional): callback function for MetaFrame. Defaults to None.
        """
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
        if isinstance(frame, DataFrame) and self.callback_data is not None:
            self.callback_data(src_address, frame)
        elif isinstance(frame, MetaFrame) and self.callback_meta is not None:
            self.callback_meta(src_address, frame)

    def send(self, target_address, frame, num_trial=1):
        if isinstance(frame, DataFrame):
            data = frame.to_bytes()
        elif isinstance(frame, MetaFrame):
            data = frame.to_bytes()
        else:
            raise ValueError(f"Unknown frame type: {type(frame)}")
        self.esp_now_ros_interface.send(target_address, data, num_trial)
