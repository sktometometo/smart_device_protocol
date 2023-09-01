import struct
from typing import List, Tuple, Union

from esp_now_ros.msg import Packet

PACKET_TYPE_META = Packet.PACKET_TYPE_META
PACKET_TYPE_DATA = Packet.PACKET_TYPE_DATA


class MetaFrame:
    def __init__(self, device_name: str, interface_descriptions: List[Tuple[str, str]]):
        self.device_name = device_name
        self.interface_descriptions = interface_descriptions

    def __repr__(self):
        output = f"device_name: {self.device_name}\n"
        output += "interface_description:\n"
        for int_des in self.interface_descriptions:
            output += f"  - [{int_des[0]}, {int_des[1]}]"
        return output

    def __eq__(self, other):
        if isinstance(other, MetaFrame):
            return self.__dict__ == other.__dict__
        return False

    __hash__ = None

    def to_bytes(self) -> bytes:
        data = struct.pack("<B", PACKET_TYPE_META)
        data += struct.pack("20s", self.device_name.encode("utf-8"))
        for i in range(3):
            if i < len(self.interface_descriptions):
                packet_description = self.interface_descriptions[i][0]
                serialization_format = self.interface_descriptions[i][1]
                data += struct.pack(
                    "64s", packet_description.encode("utf-8")
                ) + struct.pack("10s", serialization_format.encode("utf-8"))
            else:
                data += b"\x00" * 74
        return data

    @staticmethod
    def from_bytes(data: bytes):
        packet_type = struct.unpack("<B", data[0:1])[0]
        if packet_type != PACKET_TYPE_META:
            raise ValueError(f"packet type if not MetaFrame: {packet_type}")
        device_name = (
            struct.unpack("20s", data[1 : 1 + 20])[0]
            .decode("utf-8")
            .replace("\x00", "")
        )
        interface_descriptions = []
        for i in range(3):
            packet_description = (
                struct.unpack("64s", data[21 + i * 74 : 21 + 64 + i * 74])[0]
                .decode("utf-8")
                .replace("\x00", "")
            )
            serialization_format = (
                struct.unpack("10s", data[21 + 64 + i * 74 : 21 + 64 + 10 + i * 74])[0]
                .decode("utf-8")
                .replace("\x00", "")
            )
            if len(packet_description) > 0 and len(serialization_format) > 0:
                interface_descriptions.append(
                    (packet_description, serialization_format)
                )
        return MetaFrame(
            device_name=device_name, interface_descriptions=interface_descriptions
        )


class DataFrame:
    def __init__(
        self, packet_description: str, content: List[Union[bool, int, float, str]]
    ):
        self.packet_description = packet_description
        serialization_format = ""
        for entry in content:
            if type(entry) is bool:
                serialization_format += "?"
            elif type(entry) is int:
                serialization_format += "i"
            elif type(entry) is float:
                serialization_format += "f"
            elif type(entry) is str:
                encoded_entry = entry.encode("utf-8")
                if len(encoded_entry) <= 16:
                    serialization_format += "s"
                elif len(encoded_entry) <= 64:
                    serialization_format += "S"
                else:
                    raise ValueError(
                        f"String entry of content is longer than 64 bytes: {len(encoded_entry)}"
                    )
            else:
                raise ValueError(f"There is an unknown type of content: {type(entry)}")
        self.serialization_format = serialization_format
        self.content = content

    def __repr__(self):
        output = f"packet_description: {self.packet_description}\n"
        output += f"serialization_format: {self.serialization_format}\n"
        output += f"content: {self.content}"
        return output

    def __eq__(self, other):
        if isinstance(other, MetaFrame):
            return self.__dict__ == other.__dict__
        return False

    def to_bytes(self) -> bytes:
        data: bytes = (
            struct.pack("<B", PACKET_TYPE_DATA)
            + struct.pack("64s", self.packet_description.encode("utf-8"))
            + struct.pack("10s", self.serialization_format.encode("utf-8"))
        )
        for format_c, entry in zip(self.serialization_format, self.content):
            if format_c == "?":
                data += struct.pack("?", entry)
            elif format_c == "i":
                data += struct.pack("<i", entry)
            elif format_c == "f":
                data += struct.pack("<f", entry)
            elif format_c == "s":
                data += struct.pack("16s", str(entry).encode("utf-8"))
            elif format_c == "S":
                data += struct.pack("64s", str(entry).encode("utf-8"))
            else:
                raise ValueError(f"Unknown format_c specifier: {format_c}")
        return data

    @staticmethod
    def from_bytes(data: bytes):
        packet_type = struct.unpack("<B", data[0:1])[0]
        if packet_type != PACKET_TYPE_DATA:
            raise ValueError(f"packet type if not DataFrame: {packet_type}")
        packet_description = (
            struct.unpack("64s", data[1 : 1 + 64])[0]
            .decode("utf-8")
            .replace("\x00", "")
        )
        serialization_format = (
            struct.unpack("10s", data[1 + 64 : 1 + 64 + 10])[0]
            .decode("utf-8")
            .replace("\x00", "")
        )
        content: List[Union[bool, int, float, str]] = []
        index: int = 1 + 64 + 10
        for format_specifier in serialization_format:
            if format_specifier == "?":
                entry = struct.unpack("<?", data[index : index + 1])[0]
                content.append(entry)
                index += 1
            elif format_specifier == "i":
                entry = struct.unpack("<i", data[index : index + 4])[0]
                content.append(entry)
                index += 4
            elif format_specifier == "f":
                entry = struct.unpack("<f", data[index : index + 4])[0]
                content.append(entry)
                index += 4
            elif format_specifier == "s":
                entry = (
                    struct.unpack("16s", data[index : index + 16])[0]
                    .decode("utf-8")
                    .replace("\x00", "")
                )
                content.append(entry)
                index += 16
            elif format_specifier == "s":
                entry = (
                    struct.unpack("64s", data[index : index + 64])[0]
                    .decode("utf-8")
                    .replace("\x00", "")
                )
                content.append(entry)
                index += 64
            else:
                raise ValueError(f"Unknown format specifier: {format_specifier}")
        return DataFrame(packet_description=packet_description, content=content)


def parse_packet(packet: Packet) -> Tuple[Tuple, Union[MetaFrame, DataFrame]]:
    src_address = struct.unpack("6B", packet.mac_address)
    packet_type = struct.unpack("<B", packet.data[0:1])[0]
    if packet_type == PACKET_TYPE_META:
        return src_address, MetaFrame.from_bytes(packet.data)
    elif packet_type == PACKET_TYPE_DATA:
        return src_address, DataFrame.from_bytes(packet.data)
    else:
        raise ValueError(f"Unknown packet type: {packet_type}")
