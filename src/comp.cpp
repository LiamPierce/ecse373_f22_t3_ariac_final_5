
#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"

std_srvs::Trigger begin_comp;
std_srvs::SetBool my_bool_var;

int main(int argc, char **argv){

  ros::init(argc, argv, "final_project_5");
  ros::NodeHandle n;

  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  begin_client.call(begin_comp);

  if (!begin_client.call(begin_comp)){
    ROS_ERROR("Competition service call failed!  Goodness Gracious!!");
    return 1;
  }

  // Sample output for service failure.
  ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
  // Sample output for service success.
  ROS_INFO("Competition service called successfully: %s", \
  begin_comp.response.message.c_str());

  ros::Rate loop_rate(100);

  int count = 0;
  while (ros::ok()){
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
