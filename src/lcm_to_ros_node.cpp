#include "lcm_to_ros.h"

int main(int argc, char** argv) {
  
  lcm::LCM lcm;
  if(!lcm.good())
    return 1;

  ros::init(argc, argv, "lcm_to_ros_node");
  mini_cheetah::LcmToRosConvertor lcm_to_ros_convertor;

  lcm.subscribe("microstrain", &mini_cheetah::LcmToRosConvertor::LcmImuCallback, &lcm_to_ros_convertor);
  lcm.subscribe("leg_control_data", &mini_cheetah::LcmToRosConvertor::LcmJointStateCallback, &lcm_to_ros_convertor);
  lcm.subscribe("wbc_lcm_data", &mini_cheetah::LcmToRosConvertor::LcmContactCallback, &lcm_to_ros_convertor);

  int lcm_timeout = 100; //ms
  while( ( lcm.handleTimeout(lcm_timeout) >= 0 ) && (ros::ok()) ) //
        ros::spinOnce();

  return 0;
}
