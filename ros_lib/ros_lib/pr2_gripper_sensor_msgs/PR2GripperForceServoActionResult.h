#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperForceServoActionResult_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperForceServoActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "pr2_gripper_sensor_msgs/PR2GripperForceServoResult.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperForceServoActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef pr2_gripper_sensor_msgs::PR2GripperForceServoResult _result_type;
      _result_type result;

    PR2GripperForceServoActionResult():
      header(),
      status(),
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "pr2_gripper_sensor_msgs/PR2GripperForceServoActionResult"; };
    const char * getMD5(){ return "2c107270a9608ca3951b3a5b1939e401"; };

  };

}
#endif
