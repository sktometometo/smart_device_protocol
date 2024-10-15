#pragma once
#include "m5stack_utils/common.h"

#define PORT_A_SERIAL_RX 22
#define PORT_A_SERIAL_TX 21

#define PORT_B_SERIAL_RX 36
#define PORT_B_SERIAL_TX 26

#define PORT_C_SERIAL_RX 16
#define PORT_C_SERIAL_TX 17

const M5StackSerialPortInfo M5StackSerialPortInfoList[] = {
    {PORT_A_SERIAL_RX, PORT_A_SERIAL_TX},
    {PORT_B_SERIAL_RX, PORT_B_SERIAL_TX},
    {PORT_C_SERIAL_RX, PORT_C_SERIAL_TX},
};