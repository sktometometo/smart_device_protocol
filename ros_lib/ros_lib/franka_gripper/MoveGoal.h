#ifndef _ROS_franka_gripper_MoveGoal_h
#define _ROS_franka_gripper_MoveGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace franka_gripper
{

  class MoveGoal : public ros::Msg
  {
    public:
      typedef float _width_type;
      _width_type width;
      typedef float _speed_type;
      _speed_type speed;

    MoveGoal():
      width(0),
      speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->width);
      offset += serializeAvrFloat64(outbuffer + offset, this->speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->width));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->speed));
     return offset;
    }

    const char * getType(){ return "franka_gripper/MoveGoal"; };
    const char * getMD5(){ return "d16050a0fe85f1c2cb8347c99678362e"; };

  };

}
#endif
