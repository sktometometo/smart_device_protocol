#pragma once

enum M5StackPort {
  PORT_A,
  PORT_B,
  PORT_C,
};

struct M5StackSerialPortInfo {
  int rx;
  int tx;
};