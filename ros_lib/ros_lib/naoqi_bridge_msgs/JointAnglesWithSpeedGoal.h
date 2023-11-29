#ifndef _ROS_naoqi_bridge_msgs_JointAnglesWithSpeedGoal_h
#define _ROS_naoqi_bridge_msgs_JointAnglesWithSpeedGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "naoqi_bridge_msgs/JointAnglesWithSpeed.h"

namespace naoqi_bridge_msgs
{

  class JointAnglesWithSpeedGoal : public ros::Msg
  {
    public:
      typedef naoqi_bridge_msgs::JointAnglesWithSpeed _joint_angles_type;
      _joint_angles_type joint_angles;

    JointAnglesWithSpeedGoal():
      joint_angles()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->joint_angles.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->joint_angles.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "naoqi_bridge_msgs/JointAnglesWithSpeedGoal"; };
    const char * getMD5(){ return "d19a898a40aae87b37b0f91c9e90f46c"; };

  };

}
#endif
