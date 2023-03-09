import struct

from esp_now_ros.msg import Packet


def create_test_packet():

  return struct.pack('H', Packet.PACKET_TYPE_TEST) + \
         struct.pack('i', -120) + \
         struct.pack('f', -1.0) + \
         struct.pack('64s', 'Hello, world!'.encode('utf-8'))


def create_task_dispatcher_packet(caller_name: str,
                                  target_name: str,
                                  task_name: str,
                                  task_args: str = ''):

  return struct.pack('H', Packet.PACKET_TYPE_TASK_DISPATCHER) + \
       struct.pack('16s', caller_name.encode('utf-8')) + \
       struct.pack('16s', target_name.encode('utf-8')) + \
       struct.pack('16s', task_name.encode('utf-8')) + \
       struct.pack('{}s'.format(max(1, len(task_args))), task_args.encode('utf-8'))


def create_emergency_packet(map_frame: str, position_x: float,
                            position_y: float, position_z: float,
                            orientation_x: float, orientation_y: float,
                            orientation_z: float, orientation_w: float):

  return struct.pack('H', Packet.PACKET_TYPE_EMERGENCY) + \
      struct.pack('64s', map_frame.encode('utf-8')) + \
      struct.pack('f', position_x) + \
      struct.pack('f', position_y) + \
      struct.pack('f', position_z) + \
      struct.pack('f', orientation_x) + \
      struct.pack('f', orientation_y) + \
      struct.pack('f', orientation_z) + \
      struct.pack('f', orientation_w)
