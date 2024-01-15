#!/bin/bash

rosbag record -O ./test_topics.bag --duration=30 \
    /smart_device_protocol/recv \
    /smart_device_protocol/send \
    /smart_device_protocol/uwb
