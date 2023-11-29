#ifndef _ROS_naoqi_bridge_msgs_BlinkFeedback_h
#define _ROS_naoqi_bridge_msgs_BlinkFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/ColorRGBA.h"

namespace naoqi_bridge_msgs
{

  class BlinkFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::ColorRGBA _last_color_type;
      _last_color_type last_color;

    BlinkFeedback():
      last_color()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->last_color.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->last_color.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "naoqi_bridge_msgs/BlinkFeedback"; };
    const char * getMD5(){ return "6f1f94fb3eb06412264f6e0c5e72cfab"; };

  };

}
#endif
