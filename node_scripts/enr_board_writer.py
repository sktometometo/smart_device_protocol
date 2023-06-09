#!/usr/bin/env python

import fire
import rospy

from esp_now_ros.esp_now_ros_interface import ESPNOWROSInterface
from esp_now_ros.packet_generator import create_device_message_board_data


def main(target_address:str, message: str, source_name: str = 'sample_source'):
  
  rospy.init_node('esp_now_packet_printer')
  interface = ESPNOWROSInterface()
  rospy.sleep(5.)
  addr = list(map(lambda x: int(x, 16), target_address.split(':')))
  packet = create_device_message_board_data(source_name=source_name, message=message)
  interface.send(addr, packet, num_trial=5)
  rospy.sleep(1.)
  rospy.loginfo(f'Send packet with message {message} from {source_name} to {target_address} Finished.')

if __name__ == '__main__':
  fire.Fire(main)

