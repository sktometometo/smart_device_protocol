#ifndef _ROS_franka_gripper_HomingFeedback_h
#define _ROS_franka_gripper_HomingFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace franka_gripper
{

  class HomingFeedback : public ros::Msg
  {
    public:

    HomingFeedback()
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

    const char * getType(){ return "franka_gripper/HomingFeedback"; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif
