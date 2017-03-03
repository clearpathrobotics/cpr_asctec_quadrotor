#ifndef ASCTEC_HARDWARE_H
#define ASCTEC_HARDWARE_H

#include "asctec_serial.h"

#include <geometry_msgs/AccelStamped.h>
#include <hardware_interface/robot_hw.h>
#include <hector_quadrotor_interface/helpers.h>
#include <hector_quadrotor_interface/limiters.h>
#include <hector_quadrotor_interface/quadrotor_interface.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <hector_uav_msgs/MotorStatus.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

#include <cstdint>

namespace cpr_asctec_driver
{

  // cribbed from https://github.com/ccny-ros-pkg/asctec_drivers/blob/master/asctec_proc/include/asctec_proc/asctec_proc.h
  const double ASC_TO_ROS_ANGLE  = (1.0 /  1000.0) * 3.14159265 / 180.0; // converts to rad
  const double ASC_TO_ROS_ANGVEL = (1.0 /    64.8) * 3.14159265 / 180.0; // convetts to rad/s
  const double ASC_TO_ROS_ACC    = (1.0 / 10000.0) * 9.81;               // converts to m/s^s
  const double ASC_TO_ROS_HEIGHT = (1.0 /  1000.0);                      // converts to m

  const double ASC_TO_ROS_LAT_LONG = 1.0 / 10000000.0;
  const uint8_t ASC_GPS_FIX = 0x03;

  class AsctecHardware : public hardware_interface::RobotHW
  {
  public:
    AsctecHardware(ros::NodeHandle nh, ros::NodeHandle private_nh);

    void requestData();

    void writeCommands(ros::Time time, ros::Duration period);

    void updateState(ros::Time time);

    bool enableMotorsCb(hector_uav_msgs::EnableMotors::Request &req, hector_uav_msgs::EnableMotors::Response &res);

    void estopCb(const std_msgs::BoolConstPtr &estop_msg);

  private:

    AsctecSerial::Ptr asctec_serial_;

    boost::mutex command_mutex_;
    boost::mutex control_mutex_;
    const CTRL_INPUT toggle_, zero_;

    double mass_;
    double inertia_[3];

    hector_quadrotor_interface::QuadrotorInterface interface_;
    std_msgs::Header header_;
    geometry_msgs::Pose pose_;
    geometry_msgs::Twist twist_;
    geometry_msgs::Accel acceleration_;
    // TODO(pbovbel) initialize frame ids:
    sensor_msgs::Imu imu_;
    sensor_msgs::MagneticField mag_;
    sensor_msgs::NavSatFix gps_fix_;
    hector_uav_msgs::MotorStatus motor_status_;

    boost::shared_ptr<hector_quadrotor_interface::AttitudeSubscriberHelper> attitude_subscriber_helper_;

    hector_quadrotor_interface::AttitudeCommandHandlePtr attitude_input_;
    hector_quadrotor_interface::YawrateCommandHandlePtr yawrate_input_;
    hector_quadrotor_interface::ThrustCommandHandlePtr thrust_input_;

    hector_uav_msgs::AttitudeCommand attitude_command_;
    hector_uav_msgs::YawrateCommand yawrate_command_;
    hector_uav_msgs::ThrustCommand thrust_command_;

    hector_quadrotor_interface::AttitudeCommandLimiter attitude_limiter_;
    hector_quadrotor_interface::YawrateCommandLimiter yawrate_limiter_;
    hector_quadrotor_interface::ThrustCommandLimiter thrust_limiter_;

    std::string base_link_frame_, world_frame_;

    double motor_enable_timeout_, state_timeout_, command_timeout_;

    ros::Subscriber estop_sub_;
    bool estop_, state_estop_;
    hector_uav_msgs::ThrustCommand estop_thrust_command_;
    double estop_deceleration_;

    double rollpitch_scale_, yaw_scale_, thrust_scale_;

    boost::shared_ptr<hector_quadrotor_interface::StateSubscriberHelper> state_sub_helper_;

    ros::Publisher motor_status_pub_, imu_pub_, mag_pub_, gps_fix_pub_;
    ros::ServiceServer motor_status_srv_;

  };


}

#endif  // ASCTEC_HARDWARE_H
