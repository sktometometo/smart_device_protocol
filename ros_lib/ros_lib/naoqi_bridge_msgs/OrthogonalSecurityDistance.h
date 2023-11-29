#ifndef _ROS_SERVICE_OrthogonalSecurityDistance_h
#define _ROS_SERVICE_OrthogonalSecurityDistance_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Float32.h"

namespace naoqi_bridge_msgs
{

static const char ORTHOGONALSECURITYDISTANCE[] = "naoqi_bridge_msgs/OrthogonalSecurityDistance";

  class OrthogonalSecurityDistanceRequest : public ros::Msg
  {
    public:
      typedef std_msgs::Float32 _orthogonal_distance_type;
      _orthogonal_distance_type orthogonal_distance;

    OrthogonalSecurityDistanceRequest():
      orthogonal_distance()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->orthogonal_distance.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->orthogonal_distance.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return ORTHOGONALSECURITYDISTANCE; };
    const char * getMD5(){ return "69e9b81707b13ae1b2bceacbb0d41137"; };

  };

  class OrthogonalSecurityDistanceResponse : public ros::Msg
  {
    public:

    OrthogonalSecurityDistanceResponse()
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

    const char * getType(){ return ORTHOGONALSECURITYDISTANCE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class OrthogonalSecurityDistance {
    public:
    typedef OrthogonalSecurityDistanceRequest Request;
    typedef OrthogonalSecurityDistanceResponse Response;
  };

}
#endif
