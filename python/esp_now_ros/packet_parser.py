import struct

from esp_now_ros.msg import Packet


def parse_packet(packet: bytes):

  packet_type = struct.unpack('H', packet[0:2])[0]

  if packet_type == Packet.PACKET_TYPE_NONE:
    return Packet.PACKET_TYPE_NONE, None

  elif packet_type == Packet.PACKET_TYPE_TEST:
    number_int = struct.unpack('i', packet[2:6])[0]
    number_float = struct.unpack('f', packet[6:10])[0]
    string = struct.unpack('64s', packet[10:74])[0].decode('utf-8').replace(
        '\x00', '')
    return packet_type, number_int, number_float, string

  elif packet_type == Packet.PACKET_TYPE_SENSOR_ENV_III:

    module_name = struct.unpack('64s', packet[2:66])[0].decode('utf-8').replace(
        '\x00', '')
    pressure = struct.unpack('i', packet[66:70])[0]
    return packet_type, module_name, pressure

  elif packet_type == Packet.PACKET_TYPE_SENSOR_UNITV2_PERSON_COUNTER:

    number_of_person = struct.unpack('i', packet[2:6])[0]
    place_name = struct.unpack('64s', packet[6:70])[0].decode('utf-8').replace(
        '\x00', '')
    return packet_type, number_of_person, place_name

  elif packet_type == Packet.PACKET_TYPE_TASK_DISPATCHER:
    caller_name = struct.unpack('16s', packet[2:18])[0].decode('utf-8').replace(
        '\x00', '')
    target_name = struct.unpack('16s',
                                packet[18:34])[0].decode('utf-8').replace(
                                    '\x00', '')
    task_name = struct.unpack('16s', packet[34:50])[0].decode('utf-8').replace(
        '\x00', '')
    if len(packet) > 50:
      task_args = struct.unpack('{}s'.format(len(packet) - 50),
                                packet[50:])[0].decode('utf-8').replace(
                                    '\x00', '')
    else:
      task_args = ''
    return packet_type, caller_name, target_name, task_name, task_args

  elif packet_type == Packet.PACKET_TYPE_TASK_RECEIVED:
    worker_name = struct.unpack('16s', packet[2:18])[0].decode('utf-8').replace(
        '\x00', '')
    caller_name = struct.unpack('16s',
                                packet[18:34])[0].decode('utf-8').replace(
                                    '\x00', '')
    task_name = struct.unpack('16s', packet[34:50])[0].decode('utf-8').replace(
        '\x00', '')
    return packet_type, worker_name, caller_name, task_name

  elif packet_type == Packet.PACKET_TYPE_TASK_RESULT:
    caller_name = struct.unpack('16s', packet[2:18])[0].decode('utf-8').replace(
        '\x00', '')
    target_name = struct.unpack('16s',
                                packet[18:34])[0].decode('utf-8').replace(
                                    '\x00', '')
    task_name = struct.unpack('16s', packet[34:50])[0].decode('utf-8').replace(
        '\x00', '')
    if len(packet) > 50:
      task_result = struct.unpack('{}s'.format(len(packet) - 50),
                                  packet[50:])[0].decode('utf-8').replace(
                                      '\x00', '')
    else:
      task_result = ''
    return packet_type, caller_name, target_name, task_name, task_result

  else:
    print('{} is not supported packet type', format(packet_type))
    return Packet.PACKET_TYPE_NONE, None
