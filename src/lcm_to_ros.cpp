#include "lcm_to_ros.h"

namespace mini_cheetah {

void LcmToRosConvertor::LcmImuCallback(const lcm::ReceiveBuffer* rbuf,
                                       const std::string& channel_name,
                                       const microstrain_lcmt* msg) {
  
  seq_imu_data_++;

  sensor_msgs::Imu imu_msg;
  imu_msg.header.seq = seq_imu_data_;
  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.frame_id = "/minicheetah/imu_data";
  imu_msg.orientation.w = msg->quat[0];
  imu_msg.orientation.x = msg->quat[1];
  imu_msg.orientation.y = msg->quat[2];
  imu_msg.orientation.z = msg->quat[3];
  imu_msg.angular_velocity.x = msg->omega[0];
  imu_msg.angular_velocity.y = msg->omega[1];
  imu_msg.angular_velocity.z = msg->omega[2];
  imu_msg.linear_acceleration.x = msg->acc[0];
  imu_msg.linear_acceleration.y = msg->acc[1];
  imu_msg.linear_acceleration.z = msg->acc[2];
  imu_publisher_.publish(imu_msg);                            
}

void LcmToRosConvertor::LcmJointStateCallback(const lcm::ReceiveBuffer* rbuf,
                                              const std::string& channel_name,
                                              const leg_control_data_lcmt* msg) {
  seq_joint_state_++;

  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header.seq = seq_joint_state_;
  joint_state_msg.header.stamp = ros::Time::now();
  joint_state_msg.header.frame_id = "/minicheetah/joint_state";
  std::vector<double> joint_position(msg->q, msg->q + sizeof(msg->q)/sizeof(msg->q[0]));
  std::vector<double> joint_velocity(msg->qd, msg->qd + sizeof(msg->qd)/sizeof(msg->qd[0]));
  std::vector<double> joint_effort(msg->tau_est, msg->tau_est + sizeof(msg->tau_est)/sizeof(msg->tau_est[0]));
  joint_state_msg.position = joint_position;
  joint_state_msg.velocity = joint_velocity;
  joint_state_msg.effort = joint_effort;
  joint_state_publisher_.publish(joint_state_msg);
}

void LcmToRosConvertor::LcmContactCallback(const lcm::ReceiveBuffer* rbuf,
                                              const std::string& channel_name,
                                              const wbc_test_data_t* msg) {
  seq_contact_++;

  inekf_msgs::ContactArray contact_msg;
  contact_msg.header.seq = seq_contact_;
  contact_msg.header.stamp = ros::Time::now();
  contact_msg.header.frame_id = "/minicheetah/contact";

  std::vector<inekf_msgs::Contact> contacts;

  for (int i = 0; i < 4; i++)
  {
    inekf_msgs::Contact ct;
    ct.id = i;
    ct.indicator =  msg->contact_est[i] > 0;
    contacts.push_back(ct);
  }
  contact_msg.contacts = contacts;
  contact_publisher_.publish(contact_msg);
}

} // mini_cheetah
