#include "cpr_asctec_driver/asctec_hardware.h"
#include "ros/ros.h"
#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"

#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

void controlLoop(cpr_asctec_driver::AsctecHardware &asctec_hardware,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time)
{

  // Calculate monotonic time difference
  time_source::time_point this_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  last_time = this_time;

  ros::Duration elapsed(elapsed_duration.count());
  ros::Time now(ros::Time::now());

  // Process control loop
  cm.update(now, elapsed);
  asctec_hardware.writeCommands(now, elapsed);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "asctec_node");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency, update_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 50.0);
  private_nh.param<double>("update_frequency", update_frequency, 5.0);

  // Initialize robot hardware and link to controller manager
  cpr_asctec_driver::AsctecHardware asctec_hardware(nh, private_nh);
  controller_manager::ControllerManager cm(&asctec_hardware, nh);

  time_source::time_point last_time = time_source::now();

  ros::CallbackQueue hardware_queue;
  ros::AsyncSpinner hardware_spinner(1, &hardware_queue), misc_spinner(2);

  ros::TimerOptions control_timer(
      ros::Duration(1 / control_frequency),
      boost::bind(controlLoop, boost::ref(asctec_hardware), boost::ref(cm), boost::ref(last_time)),
      &hardware_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);

  ros::TimerOptions update_timer(
      ros::Duration(1 / update_frequency),
      boost::bind(&cpr_asctec_driver::AsctecHardware::requestData, &asctec_hardware),
      &hardware_queue);
  ros::Timer update_loop = nh.createTimer(update_timer);

  hardware_spinner.start();
  misc_spinner.start();

  ros::Rate r(update_frequency * 10);
  while(ros::ok()){
    asctec_hardware.updateState(ros::Time::now());
    r.sleep();
  }

  return 0;
}
