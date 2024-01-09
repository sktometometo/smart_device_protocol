#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperReleaseFeedback_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperReleaseFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_gripper_sensor_msgs/PR2GripperReleaseData.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperReleaseFeedback : public ros::Msg
  {
    public:
      typedef pr2_gripper_sensor_msgs::PR2GripperReleaseData _data_type;
      _data_type data;

    PR2GripperReleaseFeedback():
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

    const char * getType(){ return "pr2_gripper_sensor_msgs/PR2GripperReleaseFeedback"; };
    const char * getMD5(){ return "b4b68d48ac7d07bdb11b7f3badfa9266"; };

  };

}
#endif
