#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperSlipServoActionFeedback_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperSlipServoActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "pr2_gripper_sensor_msgs/PR2GripperSlipServoFeedback.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperSlipServoActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef pr2_gripper_sensor_msgs::PR2GripperSlipServoFeedback _feedback_type;
      _feedback_type feedback;

    PR2GripperSlipServoActionFeedback():
      header(),
      status(),
      feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "pr2_gripper_sensor_msgs/PR2GripperSlipServoActionFeedback"; };
    const char * getMD5(){ return "8819de47d9e7dfd2acefc052395548ad"; };

  };

}
#endif
