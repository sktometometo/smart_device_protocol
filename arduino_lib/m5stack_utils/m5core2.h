#pragma once
#include "m5stack_utils/common.h"

#define PORT_A_SERIAL_RX 33
#define PORT_A_SERIAL_TX 32

#define PORT_B_SERIAL_RX 26
#define PORT_B_SERIAL_TX 36

#define PORT_C_SERIAL_RX 13
#define PORT_C_SERIAL_TX 14

const M5StackSerialPortInfo M5StackSerialPortInfoList[] = {
    {PORT_A_SERIAL_RX, PORT_A_SERIAL_TX},
    {PORT_B_SERIAL_RX, PORT_B_SERIAL_TX},
    {PORT_C_SERIAL_RX, PORT_C_SERIAL_TX},
};