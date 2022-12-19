#!/usr/bin/env python

import struct

import rospy
import tf2_ros

from esp_now_ros.msg import Packet
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse


class EmergencyCaller(object):

    def __init__(self):

        self.fixed_frame_id = rospy.get_param('~fixed_frame_id', 'map')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')

        self.pub = rospy.Publisher('/esp_now_ros/send', Packet, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.srv = rospy.Service('/call_ambulance', Trigger, self.handler)

    def handler(self, req):

        self.broadcast_emergency()
        return TriggerResponse(success=True)

    def broadcast_emergency(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame_id,
                self.fixed_frame_id,
                rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('Error: {}'.format(e))
            return

        msg = Packet()
        msg.mac_address = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]

        bytes_packet_type = [Packet.PACKET_TYPE_EMERGENCY]
        bytes_map_frame = map(
            ord, (transform.header.frame_id.encode('utf-8') + '\0' * 64)[:64])
        bytes_position_x = map(ord, struct.pack(
            '<f', transform.transform.translation.x))
        bytes_position_y = map(ord, struct.pack(
            '<f', transform.transform.translation.y))
        bytes_position_z = map(ord, struct.pack(
            '<f', transform.transform.translation.z))
        bytes_rotation_x = map(ord, struct.pack(
            '<f', transform.transform.rotation.x))
        bytes_rotation_y = map(ord, struct.pack(
            '<f', transform.transform.rotation.y))
        bytes_rotation_z = map(ord, struct.pack(
            '<f', transform.transform.rotation.z))
        bytes_rotation_w = map(ord, struct.pack(
            '<f', transform.transform.rotation.w))

        msg.data = bytes_packet_type + bytes_map_frame + \
            bytes_position_x + bytes_position_y + bytes_position_z + \
            bytes_rotation_x + bytes_rotation_y + \
            bytes_rotation_z + bytes_rotation_w

        self.pub.publish(msg)


if __name__ == '__main__':

    rospy.init_node('esp_now_emergency_caller')
    node = EmergencyCaller()
    rospy.spin()
