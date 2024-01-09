#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperSlipServoResult_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperSlipServoResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_gripper_sensor_msgs/PR2GripperSlipServoData.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperSlipServoResult : public ros::Msg
  {
    public:
      typedef pr2_gripper_sensor_msgs::PR2GripperSlipServoData _data_type;
      _data_type data;

    PR2GripperSlipServoResult():
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

    const char * getType(){ return "pr2_gripper_sensor_msgs/PR2GripperSlipServoResult"; };
    const char * getMD5(){ return "1b10af616c7e94f609790b12cde04c6d"; };

  };

}
#endif
