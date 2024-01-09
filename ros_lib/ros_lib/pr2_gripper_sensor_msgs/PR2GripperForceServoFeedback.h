#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperForceServoFeedback_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperForceServoFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_gripper_sensor_msgs/PR2GripperForceServoData.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperForceServoFeedback : public ros::Msg
  {
    public:
      typedef pr2_gripper_sensor_msgs::PR2GripperForceServoData _data_type;
      _data_type data;

    PR2GripperForceServoFeedback():
      data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->data.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->data.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "pr2_gripper_sensor_msgs/PR2GripperForceServoFeedback"; };
    const char * getMD5(){ return "a85c0d43537b45945527f5de565ab7c2"; };

  };

}
#endif
