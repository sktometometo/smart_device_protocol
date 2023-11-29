#ifndef _ROS_naoqi_bridge_msgs_BodyPoseWithSpeedFeedback_h
#define _ROS_naoqi_bridge_msgs_BodyPoseWithSpeedFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_bridge_msgs
{

  class BodyPoseWithSpeedFeedback : public ros::Msg
  {
    public:

    BodyPoseWithSpeedFeedback()
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

    const char * getType(){ return "naoqi_bridge_msgs/BodyPoseWithSpeedFeedback"; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif
