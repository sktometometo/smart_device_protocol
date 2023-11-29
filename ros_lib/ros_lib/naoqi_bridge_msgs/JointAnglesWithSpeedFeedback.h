#ifndef _ROS_naoqi_bridge_msgs_JointAnglesWithSpeedFeedback_h
#define _ROS_naoqi_bridge_msgs_JointAnglesWithSpeedFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_bridge_msgs
{

  class JointAnglesWithSpeedFeedback : public ros::Msg
  {
    public:

    JointAnglesWithSpeedFeedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return "naoqi_bridge_msgs/JointAnglesWithSpeedFeedback"; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif
