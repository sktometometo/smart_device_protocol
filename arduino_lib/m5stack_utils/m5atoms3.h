#pragma once
#include "m5stack_utils/common.h"

#define PORT_A_SERIAL_RX 1
#define PORT_A_SERIAL_TX 2

const M5StackSerialPortInfo M5StackSerialPortInfoList[] = {
    {PORT_A_SERIAL_RX, PORT_A_SERIAL_TX},
};