#pragma once

// ROS related
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "inekf_msgs/ContactArray.h"

// LCM related
#include <lcm/lcm-cpp.hpp>
#include "lcm-types/microstrain_lcmt.hpp"
#include "lcm-types/leg_control_data_lcmt.hpp"
#include "lcm-types/wbc_test_data_t.hpp"

namespace mini_cheetah {

class LcmToRosConvertor {
  public:
    LcmToRosConvertor() : nh_() {
      seq_imu_data_ = 0;
      seq_joint_state_ = 0;
      seq_contact_ = 0;

      imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("Imu", 1);
      joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("JointState", 1);
      contact_publisher_ = nh_.advertise<inekf_msgs::ContactArray>("Contacts", 1);
    }

    void LcmImuCallback(const lcm::ReceiveBuffer* rbuf,
                        const std::string& channel_name,
                        const microstrain_lcmt* msg);
    
    void LcmJointStateCallback(const lcm::ReceiveBuffer* rbuf,
                               const std::string& channel_name,
                               const leg_control_data_lcmt* msg);

    void LcmContactCallback(const lcm::ReceiveBuffer* rbuf,
                               const std::string& channel_name,
                               const wbc_test_data_t* msg);
  
  private:
    ros::NodeHandle nh_;
    ros::Publisher imu_publisher_;
    ros::Publisher joint_state_publisher_;
    ros::Publisher contact_publisher_;
    int seq_imu_data_;
    int seq_joint_state_;
    int seq_contact_;

};

} // namespace mini_cheetah
