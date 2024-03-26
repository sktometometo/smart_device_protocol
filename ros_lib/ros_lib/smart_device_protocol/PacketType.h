#ifndef _ROS_smart_device_protocol_PacketType_h
#define _ROS_smart_device_protocol_PacketType_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace smart_device_protocol
{

  class PacketType : public ros::Msg
  {
    public:
      enum { PACKET_TYPE_NONE =  0 };
      enum { PACKET_TYPE_TEST =  1 };
      enum { PACKET_TYPE_NAMED_STRING =  11 };
      enum { PACKET_TYPE_NAMED_INT =  12 };
      enum { PACKET_TYPE_NAMED_FLOAT =  13 };
      enum { PACKET_TYPE_SENSOR_ENV_III =  21 };
      enum { PACKET_TYPE_SENSOR_UNITV2_PERSON_COUNTER =  22 };
      enum { PACKET_TYPE_EMERGENCY =  31 };
      enum { PACKET_TYPE_TASK_DISPATCHER =  32 };
      enum { PACKET_TYPE_TASK_RESULT =  33 };
      enum { PACKET_TYPE_TASK_RECEIVED =  34 };
      enum { PACKET_TYPE_DEVICE_MESSAGE_BOARD_META =  41 };
      enum { PACKET_TYPE_DEVICE_MESSAGE_BOARD_DATA =  42 };
      enum { PACKET_TYPE_META =  81 };
      enum { PACKET_TYPE_DATA =  82 };
      enum { PACKET_TYPE_RPC_META =  83 };

    PacketType()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return "smart_device_protocol/PacketType"; };
    virtual const char * getMD5() override { return "39af3ad00e88746ecbdff7da1bde3e03"; };

  };

}
#endif
