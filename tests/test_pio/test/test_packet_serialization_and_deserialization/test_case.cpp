#include <sdp/packet_creator.h>
#include <sdp/packet_parser.h>

#include "gtest/gtest.h"

void test_meta_frame_serialization_and_deserialization(void) {
  uint8_t packet[240];
  std::string device_name = "test_device";
  std::string packet_description_01 = "test_packet_01";
  std::string serialization_format_01 = "i";
  std::string packet_description_02 = "test_packet_02";
  std::string serialization_format_02 = "f";
  std::string packet_description_03 = "test_packet_03";
  std::string serialization_format_03 = "s";
  SDPInterfaceDescription interface_01 = std::make_tuple(packet_description_01, serialization_format_01);
  SDPInterfaceDescription interface_02 = std::make_tuple(packet_description_02, serialization_format_02);
  SDPInterfaceDescription interface_03 = std::make_tuple(packet_description_03, serialization_format_03);
  //
  generate_meta_frame(packet, device_name, packet_description_01, serialization_format_01, packet_description_02, serialization_format_02, packet_description_03, serialization_format_03);
  auto result = parse_packet_as_meta_packet(packet);
  std::string device_name_result = std::get<0>(result);
  std::vector<SDPInterfaceDescription> interfaces = std::get<1>(result);
  SDPInterfaceDescription interface_01_result = interfaces[0];
  SDPInterfaceDescription interface_02_result = interfaces[1];
  SDPInterfaceDescription interface_03_result = interfaces[2];
  std::string packet_description_01_result = std::get<0>(interface_01_result);
  std::string serialization_format_01_result = std::get<1>(interface_01_result);
  std::string packet_description_02_result = std::get<0>(interface_02_result);
  std::string serialization_format_02_result = std::get<1>(interface_02_result);
  std::string packet_description_03_result = std::get<0>(interface_03_result);
  std::string serialization_format_03_result = std::get<1>(interface_03_result);
  EXPECT_STREQ(device_name.c_str(), device_name_result.c_str());
  EXPECT_STREQ(packet_description_01.c_str(), packet_description_01_result.c_str());
  EXPECT_STREQ(serialization_format_01.c_str(), serialization_format_01_result.c_str());
  EXPECT_STREQ(packet_description_02.c_str(), packet_description_02_result.c_str());
  EXPECT_STREQ(serialization_format_02.c_str(), serialization_format_02_result.c_str());
  EXPECT_STREQ(packet_description_03.c_str(), packet_description_03_result.c_str());
  EXPECT_STREQ(serialization_format_03.c_str(), serialization_format_03_result.c_str());
}