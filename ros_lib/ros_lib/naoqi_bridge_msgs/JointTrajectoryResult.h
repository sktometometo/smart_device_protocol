#ifndef _ROS_naoqi_bridge_msgs_JointTrajectoryResult_h
#define _ROS_naoqi_bridge_msgs_JointTrajectoryResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/JointState.h"

namespace naoqi_bridge_msgs
{

  class JointTrajectoryResult : public ros::Msg
  {
    public:
      typedef sensor_msgs::JointState _goal_position_type;
      _goal_position_type goal_position;

    JointTrajectoryResult():
      goal_position()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->goal_position.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->goal_position.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "naoqi_bridge_msgs/JointTrajectoryResult"; };
    const char * getMD5(){ return "1c77b3d9dc137611510fd16c3b792046"; };

  };

}
#endif
