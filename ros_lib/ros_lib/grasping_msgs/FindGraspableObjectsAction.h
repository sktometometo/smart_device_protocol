#ifndef _ROS_grasping_msgs_FindGraspableObjectsAction_h
#define _ROS_grasping_msgs_FindGraspableObjectsAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "grasping_msgs/FindGraspableObjectsActionGoal.h"
#include "grasping_msgs/FindGraspableObjectsActionResult.h"
#include "grasping_msgs/FindGraspableObjectsActionFeedback.h"

namespace grasping_msgs
{

  class FindGraspableObjectsAction : public ros::Msg
  {
    public:
      typedef grasping_msgs::FindGraspableObjectsActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef grasping_msgs::FindGraspableObjectsActionResult _action_result_type;
      _action_result_type action_result;
      typedef grasping_msgs::FindGraspableObjectsActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    FindGraspableObjectsAction():
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

    const char * getType(){ return "grasping_msgs/FindGraspableObjectsAction"; };
    const char * getMD5(){ return "ee328bdfce4619bf201b406a666b5877"; };

  };

}
#endif
