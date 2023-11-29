#ifndef _ROS_mongodb_store_msgs_MoveEntriesAction_h
#define _ROS_mongodb_store_msgs_MoveEntriesAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mongodb_store_msgs/MoveEntriesActionGoal.h"
#include "mongodb_store_msgs/MoveEntriesActionResult.h"
#include "mongodb_store_msgs/MoveEntriesActionFeedback.h"

namespace mongodb_store_msgs
{

  class MoveEntriesAction : public ros::Msg
  {
    public:
      typedef mongodb_store_msgs::MoveEntriesActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef mongodb_store_msgs::MoveEntriesActionResult _action_result_type;
      _action_result_type action_result;
      typedef mongodb_store_msgs::MoveEntriesActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    MoveEntriesAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "mongodb_store_msgs/MoveEntriesAction"; };
    const char * getMD5(){ return "603d33caf9a321e4af460957d0a9266a"; };

  };

}
#endif
