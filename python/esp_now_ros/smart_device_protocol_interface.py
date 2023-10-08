from operator import call
import rospy

import struct
from typing import Callable, Union, List, Tuple, Optional, Dict

from esp_now_ros.sdp_frames import MetaFrame, DataFrame, BaseFrame
from esp_now_ros.esp_now_ros_interface import ESPNOWROSInterface
from esp_now_ros.packet_parser import parse_packet_as_v2
from esp_now_ros.msg import Packet, UWBDistance


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
        self._callback_data = callback_data
        self._callback_meta = callback_meta
        self._esp_now_ros_interface = ESPNOWROSInterface(self._callback)

    def _callback(self, src_address, data):
        try:
            _, frame = parse_packet_as_v2(
                Packet(data=data, mac_address=struct.pack("6B", *list(src_address)))
            )
        except ValueError:
            return
        if isinstance(frame, DataFrame) and self._callback_data is not None:
            self._callback_data(src_address, frame)
        elif isinstance(frame, MetaFrame) and self._callback_meta is not None:
            self._callback_meta(src_address, frame)

    def send(self, target_address, frame, num_trial=1):
        if isinstance(frame, DataFrame):
            data = frame.to_bytes()
        elif isinstance(frame, MetaFrame):
            data = frame.to_bytes()
        else:
            raise ValueError(f"Unknown frame type: {type(frame)}")
        self._esp_now_ros_interface.send(target_address, data, num_trial)


class DeviceDictSDPInterface(SDPInterface):
    """Smart Device Protocol Interface with Device Dictionary"""

    def __init__(self, callback_data=None, callback_meta=None):
        self._device_interfaces: Dict[Union[List[int], Tuple[int]], Dict] = {}
        self._original_callback_data = callback_data
        self._original_callback_meta = callback_meta
        super().__init__(
            self._original_callback_data, self._callback_meta_for_device_interfaces
        )

    def _callback_meta_for_device_interfaces(self, src_address, frame):
        self._device_interface_meta_callback(src_address, frame)
        if self._original_callback_meta is not None:
            self._original_callback_meta(src_address, frame)

    def _device_interface_meta_callback(self, src_address, frame):
        if src_address not in self._device_interfaces:
            self._device_interfaces[src_address] = {}
            self._device_interfaces[src_address]["device_name"] = frame.device_name
            self._device_interfaces[src_address]["interfaces"] = []
            self._device_interfaces[src_address]["last_stamp"] = rospy.Time.now()

        for i in range(3):
            if (frame.interface_descriptions[i] != ("", "")) and (
                frame.interface_description[i]
                not in self._device_interfaces[src_address]["interfaces"]
            ):
                self._device_interfaces[src_address]["interfaces"].append(
                    frame.interface_descriptions[i]
                )

    @property
    def device_interfaces(self):
        return self._device_interfaces


class DeviceDictSDPInterfaceWithInterfaceCallback(DeviceDictSDPInterface):
    def __init__(
        self, callbacks_data: Dict[Tuple[str, str], Callable] = {}, callback_meta=None
    ):
        self._callbacks_data = callbacks_data
        self._callback_meta = callback_meta
        super().__init__(self._callback_data_for_interface, self._callback_meta)

    def _callback_data_for_interface(self, src_address, frame):
        interface_description = frame.interface_description
        if interface_description in self._callbacks_data:
            self._callbacks_data[interface_description](src_address, frame)

    def register_callback(self, interface_description, callback):
        self._callbacks_data[interface_description] = callback

    def unregister_callback(self, interface_description):
        del self._callbacks_data[interface_description]
