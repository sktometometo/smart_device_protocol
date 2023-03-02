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
  else:
    print('{} is not supported packet type', format(packet_type))


def create_test_packet():

  return struct.pack('H', Packet.PACKET_TYPE_TEST) + struct.pack(
      'i', -120) + struct.pack('f', -1.0) + struct.pack(
          '64s', 'Hello, world!'.encode('utf-8'))


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