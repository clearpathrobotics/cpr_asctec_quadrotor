#include "cpr_asctec_driver/asctec_hardware.h"
#include "cpr_asctec_driver/asctec_serial.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "hector_quadrotor_interface/helpers.h"
#include <ros/ros.h>

#include <cstdint>

namespace cpr_asctec_driver
{

  using namespace hector_quadrotor_interface;

  AsctecHardware::AsctecHardware(ros::NodeHandle nh, ros::NodeHandle private_nh)
      : toggle_(0, 0, -2047, 0), zero_(0, 0, 0, 0)
  {

    this->registerInterface(&interface_);
    interface_.registerAccel(&acceleration_);
    interface_.registerPose(&pose_);
    interface_.registerMotorStatus(&motor_status_);
    interface_.registerSensorImu(&imu_);
    interface_.registerTwist(&twist_);

    std::string port;
    if(!private_nh.getParam("port", port)){
      ROS_ERROR("Missing required 'port' parameter");
      throw new std::invalid_argument("Missing required 'port' parameter");
    }
    asctec_serial_.reset(new AsctecSerial(port, 57600));

    nh.param<double>("motor_enable_timeout", motor_enable_timeout_, 2.0);
    nh.param<double>("state_timeout", state_timeout_, 0.05);
    nh.param<double>("estop_deceleration", estop_deceleration_, 1.0);

    nh.param<double>("rollpitch_scale", rollpitch_scale_, 3000.0);
    nh.param<double>("yaw_scale", yaw_scale_, 400.0);
    nh.param<double>("thrust_scale", thrust_scale_, 780.0);

    motor_status_pub_ = nh.advertise<hector_uav_msgs::MotorStatus>("motor_status", 1);
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data", 1);
    mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
    gps_fix_pub_ = nh.advertise<sensor_msgs::NavSatFix>("fix", 1);
    motor_status_srv_ = nh.advertiseService("enable_motors", &AsctecHardware::enableMotorsCb, this);

    ros::NodeHandle limit_nh(nh, "limits");
    attitude_limiter_.init(limit_nh, "pose/orientation");
    yawrate_limiter_.init(limit_nh, "twist/angular");
    thrust_limiter_.init(limit_nh, "wrench/force");

    getMassAndInertia(nh, mass_, inertia_);

    state_sub_helper_ = boost::make_shared<StateSubscriberHelper>(nh, "state", boost::ref(pose_),
                                                                   boost::ref(twist_), boost::ref(acceleration_),
                                                                   boost::ref(header_));

    // subscribe to attitude, yawrate, and thrust
    attitude_subscriber_helper_ = boost::make_shared<AttitudeSubscriberHelper>(ros::NodeHandle(nh, "command"),
                                                                               boost::ref(command_mutex_),
                                                                               boost::ref(attitude_command_),
                                                                               boost::ref(yawrate_command_),
                                                                               boost::ref(thrust_command_));
    attitude_command_.pitch = 0;
    attitude_command_.roll = 0;
    yawrate_command_.turnrate = 0;
    thrust_command_.thrust = 0;

    attitude_input_ = interface_.addInput<AttitudeCommandHandle>("attitude");
    yawrate_input_ = interface_.addInput<YawrateCommandHandle>("yawrate");
    thrust_input_ = interface_.addInput<ThrustCommandHandle>("thrust");


    estop_ = false;
    state_estop_ = false;
    estop_sub_ = nh.subscribe("estop", 1, &AsctecHardware::estopCb, this);
  }

  bool AsctecHardware::enableMotorsCb(hector_uav_msgs::EnableMotors::Request &req, hector_uav_msgs::EnableMotors::Response &res)
  {
    // Enable motors service has exclusive lock on control
    boost::mutex::scoped_lock lock(control_mutex_);

    ros::Time timeout = ros::Time::now() + ros::Duration(motor_enable_timeout_);
    ros::Rate r(10.0);

    while (motor_status_.running != req.enable) {
      asctec_serial_->sendCommand(toggle_);
      if (ros::Time::now() > timeout) {
        ROS_INFO_STREAM("Couldn't " << (req.enable ? "enable" : "disable") << " motors for " << motor_enable_timeout_ << "s, giving up");
        break;
      }
      r.sleep();
    }

    res.success = (motor_status_.running == req.enable);
    return true;
  }

  void AsctecHardware::requestData()
  {
    // TODO parametrize rate of individual components
    std::set<AsctecSerial::DataType> requests;
    requests.insert(AsctecSerial::STATUS);
    requests.insert(AsctecSerial::IMU_CALC);
    requests.insert(AsctecSerial::GPS);

    asctec_serial_->requestData(requests);
  }

  void AsctecHardware::writeCommands(ros::Time time, ros::Duration period)
  {

    if(time > header_.stamp + ros::Duration(state_timeout_)){
      if(!state_estop_){
        estop_thrust_command_ = thrust_command_;
      }
      ROS_WARN_STREAM_THROTTLE(1.0, "No state information received for at least " << state_timeout_ << "s, triggering estop");
      state_estop_ = true;
    }else if(state_estop_){
      state_estop_ = false;
    }

    boost::mutex::scoped_lock control_lock(control_mutex_, boost::try_to_lock);
    if (control_lock) {

      // Lock around command access
      boost::mutex::scoped_lock command_lock(command_mutex_);

      if (attitude_input_->connected() && attitude_input_->enabled()) {
        attitude_command_ = attitude_input_->getCommand();
      }
      if (yawrate_input_->connected() && yawrate_input_->enabled()) {
        yawrate_command_ = yawrate_input_->getCommand();
      }
      if (thrust_input_->connected() && thrust_input_->enabled()) {
        thrust_command_ = thrust_input_->getCommand();
      }

      attitude_command_ = attitude_limiter_.limit(attitude_command_);
      yawrate_command_ = yawrate_limiter_.limit(yawrate_command_);
      thrust_command_ = thrust_limiter_.limit(thrust_command_);

      if(estop_ || state_estop_){
        attitude_command_.roll = attitude_command_.pitch = yawrate_command_.turnrate = 0;

        estop_thrust_command_.thrust = std::max(estop_thrust_command_.thrust - estop_deceleration_ * mass_ * period.toSec(), 0.0);
        thrust_command_ = estop_thrust_command_;
      }

      short roll = attitude_command_.roll * rollpitch_scale_; //roll input: -2047..+2047 (0=neutral)
      roll = std::max(std::min(roll, static_cast<short>(2047)), static_cast<short>(-2047));

      short pitch = -attitude_command_.pitch * rollpitch_scale_; //pitch input: -2047..+2047 (0=neutral)
      pitch = std::max(std::min(pitch, static_cast<short>(2047)), static_cast<short>(-2047));

      short yaw = -yawrate_command_.turnrate * yaw_scale_; //(=R/C Stick input) -2047..+2047 (0=neutral)
      yaw = std::max(std::min(yaw, static_cast<short>(2047)), static_cast<short>(-2047));

      short thrust = std::pow(thrust_command_.thrust, 0.5) * thrust_scale_; //collective: 0..4095 = 0..100%
      thrust = std::max(std::min(thrust, static_cast<short>(4095)), static_cast<short>(0));

      // TODO(pbovbel) better formatting for a 'limited command debug' here
      ROS_DEBUG_STREAM("roll " << roll);
      ROS_DEBUG_STREAM("pitch  " << pitch);
      ROS_DEBUG_STREAM("yaw " << yaw);
      ROS_DEBUG_STREAM("thrust " << thrust);

      asctec_serial_->sendCommand(CTRL_INPUT(roll, pitch, yaw, thrust));
    }

  }

  void AsctecHardware::updateState(ros::Time time)
  {
    AsctecSerial::DataType data = asctec_serial_->poll();

    switch (data) {
      case AsctecSerial::NONE:
      {
        ROS_DEBUG("Timed out waiting for message from hardware.");
        break;
      }
      case AsctecSerial::STATUS:
      {
        motor_status_.header.stamp = time;
        motor_status_.on = asctec_serial_->status.motors_on;
        motor_status_.running = asctec_serial_->status.flying;
        motor_status_pub_.publish(motor_status_);
        break;
      }
      case AsctecSerial::IMU_CALC:
      {
        imu_.header.stamp = time;
        tf2::Quaternion q;
        q.setRPY(
          static_cast<float>(asctec_serial_->imu_calc.angle_roll)  * ASC_TO_ROS_ANGLE * -1.0,
          static_cast<float>(asctec_serial_->imu_calc.angle_nick)  * ASC_TO_ROS_ANGLE *  1.0,
          static_cast<float>(asctec_serial_->imu_calc.angle_yaw)   * ASC_TO_ROS_ANGLE * -1.0
        );
        imu_.orientation = tf2::toMsg(q);

        imu_.angular_velocity.x = asctec_serial_->imu_calc.angvel_roll * ASC_TO_ROS_ANGVEL *  1.0;
        imu_.angular_velocity.y = asctec_serial_->imu_calc.angvel_nick * ASC_TO_ROS_ANGVEL *  1.0;
        imu_.angular_velocity.z = asctec_serial_->imu_calc.angvel_yaw  * ASC_TO_ROS_ANGVEL * -1.0;

        imu_.linear_acceleration.x = asctec_serial_->imu_calc.acc_x_calib * ASC_TO_ROS_ACC * 1.0;
        imu_.linear_acceleration.y = asctec_serial_->imu_calc.acc_y_calib * ASC_TO_ROS_ACC * -1.0;
        imu_.linear_acceleration.z = asctec_serial_->imu_calc.acc_z_calib * ASC_TO_ROS_ACC * -1.0;
        imu_pub_.publish(imu_);

        mag_.header.stamp = time;
        mag_.magnetic_field.x = asctec_serial_->imu_calc.Hx *  1.0;
        mag_.magnetic_field.y = asctec_serial_->imu_calc.Hy *  1.0;
        mag_.magnetic_field.z = asctec_serial_->imu_calc.Hz * -1.0;
        mag_pub_.publish(mag_);
        break;
      }
      case AsctecSerial::IMU_RAW:
      {
        // no-op, not implemented
        break;
      }
      case AsctecSerial::GPS:
      {
        gps_fix_.header.stamp = time;
        gps_fix_.status.status = static_cast<uint8_t>(asctec_serial_->gps.status) == ASC_GPS_FIX ?
          sensor_msgs::NavSatStatus::STATUS_FIX : sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        gps_fix_.latitude = asctec_serial_->gps.latitude * ASC_TO_ROS_LAT_LONG;
        gps_fix_.longitude = asctec_serial_->gps.longitude * ASC_TO_ROS_LAT_LONG;
        gps_fix_.altitude = asctec_serial_->gps.height * ASC_TO_ROS_HEIGHT;
        gps_fix_pub_.publish(gps_fix_);
        break;
      }
      case AsctecSerial::UNKNOWN:
      {
        ROS_ERROR_STREAM("Unknown message type " << data);
        break;
      }
    }

  }

  void AsctecHardware::estopCb(const std_msgs::BoolConstPtr &estop_msg)
  {
    bool estop = static_cast<bool>(estop_msg->data);
    if (estop_ == false && estop == true)
    {
      estop_thrust_command_ = thrust_command_;
    }
    estop_ = estop;
  }
}
