#!/usr/bin/env python

import fire
import rospy

from esp_now_ros.esp_now_ros_interface import ESPNOWROSInterface
from esp_now_ros.packet_generator import create_device_message_board_data


def main(
    target_address: str,
    message: str,
    source_name: str = "sample_source",
    timeout_duration: int = 60000,
):
    rospy.init_node("esp_board_writer")
    interface = ESPNOWROSInterface()
    rospy.sleep(5.0)
    addr = list(map(lambda x: int(x, 16), target_address.split(":")))
    packet = create_device_message_board_data(
        source_name=source_name, timeout_duration=timeout_duration, message=message
    )
    interface.send(addr, packet, num_trial=1)
    rospy.sleep(1.0)
    rospy.loginfo(
        f"Send packet with message {message} from {source_name} to {target_address} Finished."
    )


if __name__ == "__main__":
    fire.Fire(main)
