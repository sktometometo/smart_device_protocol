#ifndef _ROS_SERVICE_CmdPoseService_h
#define _ROS_SERVICE_CmdPoseService_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose2D.h"

namespace naoqi_bridge_msgs
{

static const char CMDPOSESERVICE[] = "naoqi_bridge_msgs/CmdPoseService";

  class CmdPoseServiceRequest : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose2D _pose_type;
      _pose_type pose;

    CmdPoseServiceRequest():
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return CMDPOSESERVICE; };
    const char * getMD5(){ return "271cb12677c4cd9bccbc642cd9258d1f"; };

  };

  class CmdPoseServiceResponse : public ros::Msg
  {
    public:

    CmdPoseServiceResponse()
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

    const char * getType(){ return CMDPOSESERVICE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class CmdPoseService {
    public:
    typedef CmdPoseServiceRequest Request;
    typedef CmdPoseServiceResponse Response;
  };

}
#endif
