import struct
import time
from typing import Callable, Dict, List, Optional, Tuple, Union
from urllib import response

import rospy
from gpg import Data
from smart_device_protocol.esp_now_ros_interface import ESPNOWROSInterface
from smart_device_protocol.msg import Packet, UWBDistance
from smart_device_protocol.packet_parser import InvalidPacketError, parse_packet_as_v2
from smart_device_protocol.sdp_frames import (
    BaseFrame,
    DataFrame,
    MetaFrame,
    RPCMetaFrame,
)


class SDPInterface:
    """Smart Device Protocol Interface"""

    def __init__(
        self,
        callbacks_data: Optional[
            List[Callable[[Union[List[int], Tuple[int]], BaseFrame], None]]
        ] = None,
        callbacks_meta: Optional[
            List[Callable[[Union[List[int], Tuple[int]], BaseFrame], None]]
        ] = None,
        callbacks_rpc_meta: Optional[
            List[Callable[[Union[List[int], Tuple[int]], BaseFrame], None]]
        ] = None,
    ):
        """Smart Device Protocol Interface"""
        self._callbacks_data = callbacks_data
        self._callbacks_meta = callbacks_meta
        self._callbacks_rpc_meta = callbacks_rpc_meta
        self._smart_device_protocol_interface = ESPNOWROSInterface(self._callback)

    def _callback(self, src_address, data):
        try:
            _, frame = parse_packet_as_v2(
                Packet(data=data, mac_address=struct.pack("6B", *list(src_address)))
            )
        except InvalidPacketError:
            rospy.logwarn("Failed to parse packet from {}".format(src_address))
            return
        if isinstance(frame, DataFrame) and self._callbacks_data is not None:
            for cb in self._callbacks_data:
                cb(src_address, frame)
        elif isinstance(frame, MetaFrame) and self._callbacks_meta is not None:
            for cb in self._callbacks_meta:
                cb(src_address, frame)
        elif isinstance(frame, RPCMetaFrame) and self._callbacks_rpc_meta is not None:
            for cb in self._callbacks_rpc_meta:
                cb(src_address, frame)

    def send(self, target_address, frame, num_trial=1):
        if isinstance(frame, DataFrame):
            data = frame.to_bytes()
        elif isinstance(frame, MetaFrame):
            data = frame.to_bytes()
        elif isinstance(frame, RPCMetaFrame):
            data = frame.to_bytes()
        else:
            raise ValueError(f"Unknown frame type: {type(frame)}")
        self._smart_device_protocol_interface.send(target_address, data, num_trial)

    def register_callback_data(
        self, callback: Callable[[Union[List[int], Tuple[int]], BaseFrame], None]
    ):
        if self._callbacks_data is None:
            self._callbacks_data = []
        self._callbacks_data.append(callback)

    def unregister_callback_data(
        self, callback: Callable[[Union[List[int], Tuple[int]], BaseFrame], None]
    ):
        if self._callbacks_data is not None and callback in self._callbacks_data:
            self._callbacks_data.remove(callback)

    def register_callback_meta(
        self, callback: Callable[[Union[List[int], Tuple[int]], BaseFrame], None]
    ):
        if self._callbacks_meta is None:
            self._callbacks_meta = []
        self._callbacks_meta.append(callback)

    def unregister_callback_meta(
        self, callback: Callable[[Union[List[int], Tuple[int]], BaseFrame], None]
    ):
        if self._callbacks_meta is not None and callback in self._callbacks_meta:
            self._callbacks_meta.remove(callback)

    def register_callback_rpc_meta(
        self, callback: Callable[[Union[List[int], Tuple[int]], BaseFrame], None]
    ):
        if self._callbacks_rpc_meta is None:
            self._callbacks_rpc_meta = []
        self._callbacks_rpc_meta.append(callback)

    def unregister_callback_rpc_meta(
        self, callback: Callable[[Union[List[int], Tuple[int]], BaseFrame], None]
    ):
        if (
            self._callbacks_rpc_meta is not None
            and callback in self._callbacks_rpc_meta
        ):
            self._callbacks_rpc_meta.remove(callback)


class DeviceDictSDPInterface(SDPInterface):
    """Smart Device Protocol Interface with Device Dictionary"""

    def __init__(self, timeout: float = 30.0, **kwargs):
        super().__init__(**kwargs)
        self._device_interfaces: Dict[Union[List[int], Tuple[int]], Dict] = {}
        self._device_rpc_interfaces: Dict[Union[List[int], Tuple[int]], Dict] = {}
        self._timeout = rospy.Duration(timeout)
        self.register_callback_meta(self._callback_meta_for_device_interfaces)
        self.register_callback_rpc_meta(self._callback_rpc_meta_for_device_interfaces)

    def _callback_rpc_meta_for_device_interfaces(self, src_address, frame):
        # Remove timeout device
        now = rospy.Time.now()
        for src_address in list(self._device_rpc_interfaces.keys()):
            device_rpc_interface = self._device_rpc_interfaces[src_address]
            if now - device_rpc_interface["last_stamp"] > self._timeout:
                rospy.logwarn(
                    "Remove timeout rpc device: {}, {}".format(
                        src_address, device_rpc_interface["device_name"]
                    )
                )
                self._device_rpc_interfaces.pop(src_address)

        # Device interface update
        if src_address not in self._device_rpc_interfaces:
            self._device_rpc_interfaces[src_address] = {}
            self._device_rpc_interfaces[src_address]["device_name"] = frame.device_name
            self._device_rpc_interfaces[src_address]["rpc_interfaces"] = []

        self._device_rpc_interfaces[src_address]["last_stamp"] = rospy.Time.now()

        interface_description_request = frame.interface_description_request
        interface_description_response = frame.interface_description_response

        if (
            interface_description_request,
            interface_description_response,
        ) not in self._device_rpc_interfaces[src_address]["rpc_interfaces"]:
            self._device_rpc_interfaces[src_address]["rpc_interfaces"].append(
                (interface_description_request, interface_description_response)
            )

    @property
    def device_rpc_interfaces(self):
        return self._device_rpc_interfaces

    def _callback_meta_for_device_interfaces(self, src_address, frame):
        # Remove timeout device
        now = rospy.Time.now()
        for src_address in list(self._device_interfaces.keys()):
            device_interface = self._device_interfaces[src_address]
            if now - device_interface["last_stamp"] > self._timeout:
                rospy.logwarn(
                    "Remove timeout device: {}, {}".format(
                        src_address, device_interface["device_name"]
                    )
                )
                self._device_interfaces.pop(src_address)

        # Device interface update
        if src_address not in self._device_interfaces:
            self._device_interfaces[src_address] = {}
            self._device_interfaces[src_address]["device_name"] = frame.device_name
            self._device_interfaces[src_address]["distance"] = None
            self._device_interfaces[src_address]["uwb_id"] = None
            self._device_interfaces[src_address]["interfaces"] = []

        self._device_interfaces[src_address]["last_stamp"] = rospy.Time.now()

        for interface_description in frame.interface_descriptions:
            if (interface_description != ("", "")) and (
                interface_description
                not in self._device_interfaces[src_address]["interfaces"]
            ):
                self._device_interfaces[src_address]["interfaces"].append(
                    interface_description
                )

    @property
    def device_interfaces(self):
        return self._device_interfaces

    def send(
        self,
        target: Union[Tuple[int], str],
        frame: Union[DataFrame, MetaFrame],
        num_trial=1,
    ):
        if isinstance(target, str):
            for src_address, device_interface in self._device_interfaces.items():
                if device_interface["device_name"] == target:
                    target_address = src_address
                    break
            else:
                raise ValueError(f"Unknown device name: {target}")
        else:
            target_address = target
        super().send(target_address, frame, num_trial)

    def rpc_call(
        self, target: Union[Tuple[int], str], frame: DataFrame, timeout: float = 10.0
    ) -> Optional[DataFrame]:
        if isinstance(target, str):
            for (
                src_address,
                device_rpc_interface,
            ) in self._device_rpc_interfaces.items():
                if device_rpc_interface["device_name"] == target:
                    target_address = src_address
                    break
            else:
                raise ValueError(f"Unknown device name: {target}")
        else:
            target_address = target

        interface_description_request = frame.interface_description
        interface_description_response = None
        for (
            rpc_interface_description_request,
            rpc_interface_description_response,
        ) in self._device_rpc_interfaces[target_address]["rpc_interfaces"]:
            if rpc_interface_description_request == interface_description_request:
                interface_description_response = rpc_interface_description_response
                break
        if interface_description_response is None:
            raise ValueError(
                f"Unknown rpc interface description request: {interface_description_request}"
            )

        response_dataframe = None

        def callback(src_address, frame):
            nonlocal response_dataframe
            if (
                src_address == target_address
                and frame.interface_description == interface_description_response
            ):
                response_dataframe = frame

        self.register_callback_data(callback)
        self.send(target=target, frame=frame, num_trial=1)

        start_time = time.time()
        while time.time() - start_time < timeout:
            if response_dataframe is not None:
                break
            time.sleep(0.1)
        self.unregister_callback_data(callback)
        return response_dataframe


class DeviceDictSDPInterfaceWithInterfaceCallback(DeviceDictSDPInterface):
    def __init__(
        self,
        interface_callbacks: Dict[
            Tuple[str, str], Callable[[Union[List[int], Tuple[int]], List], None]
        ] = {},
        timeout: float = 30.0,
        **kwargs,
    ):
        self._interface_callbacks = interface_callbacks
        super().__init__(timeout=timeout, **kwargs)

    def _callback_data_for_interface(self, src_address, frame):
        interface_description = frame.interface_description
        if interface_description in self._interface_callbacks:
            self._interface_callbacks[interface_description](src_address, frame)

    def register_interface_callback(
        self,
        interface_description: Tuple[str, str],
        callback: Callable[[Union[List[int], Tuple[int]], List], None],
    ):
        self._interface_callbacks[interface_description] = callback

    def unregister_interface_callback(self, interface_description: Tuple[str, str]):
        del self._interface_callbacks[interface_description]


class UWBSDPInterface(DeviceDictSDPInterfaceWithInterfaceCallback):
    def __init__(
        self,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.register_interface_callback(
            ("UWB Station", "i"), self._interface_callback_to_uwb
        )
        self.sub = rospy.Subscriber(
            "/smart_device_protocol/uwb", UWBDistance, self._cb_uwb
        )

    def _cb_uwb(self, msg: UWBDistance):
        uwb_id = msg.id
        distance = msg.distance
        for src_address, device_interface in self.device_interfaces.items():
            if device_interface["uwb_id"] == uwb_id:
                self._device_interfaces[src_address]["distance"] = distance
                break

    def _interface_callback_to_uwb(self, src_address, frame: DataFrame):
        if src_address in self._device_interfaces:
            self._device_interfaces[src_address]["uwb_id"] = frame.content[0]
