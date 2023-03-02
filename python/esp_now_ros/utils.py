import rospy
from esp_now_ros.msg import Packet
import struct


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


class ESPNOWROSInterface:

  def __init__(self, callback=None):

    self.raw_callback = callback
    self.sub = rospy.Subscriber('/esp_now_ros/recv', Packet, self.callback)
    self.pub = rospy.Publisher('/esp_now_ros/send', Packet, queue_size=1)

  def callback(self, msg):

    src_address = struct.unpack('6B', msg.mac_address)
    data = msg.data
    if self.raw_callback is not None:
      self.raw_callback(src_address, data)

  def send(self, target_address, data):
    """
    Args:
        target_address (list of int)
        data (bytes)
    """
    msg = Packet()
    msg.mac_address = struct.pack('6B', target_address[0], target_address[1],
                                  target_address[2], target_address[3],
                                  target_address[4], target_address[5])
    msg.data = data
    self.pub.publish(msg)
