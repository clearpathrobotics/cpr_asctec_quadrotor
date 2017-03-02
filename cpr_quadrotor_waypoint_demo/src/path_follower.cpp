#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "actionlib/client/simple_action_client.h"

#include <hector_uav_msgs/TakeoffAction.h>
#include <hector_uav_msgs/LandingAction.h>
#include <hector_uav_msgs/PoseAction.h>

typedef actionlib::SimpleActionClient<hector_uav_msgs::LandingAction> LandingClient;
typedef actionlib::SimpleActionClient<hector_uav_msgs::TakeoffAction> TakeoffClient;
typedef actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> PoseClient;

typedef boost::shared_ptr<PoseClient> PoseClientPtr;
typedef boost::shared_ptr<TakeoffClient> TakeoffClientPtr;
typedef boost::shared_ptr<LandingClient> LandingClientPtr;

typedef std::pair<PoseClientPtr, int> ClientState;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_follower");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  double scale, base_height;

  int repeat;

  private_nh.param<double>("scale", scale, 1.0);
  private_nh.param<double>("base_height", base_height, 1.0);
  private_nh.param<int>("repeat", repeat, 1);

  XmlRpc::XmlRpcValue path_list;
  private_nh.getParam("path", path_list);

  std::vector<geometry_msgs::Pose> pose_vector;

  for (int i = 0; i < path_list.size(); i++)
  {
    geometry_msgs::Pose pose;
    pose.position.x = static_cast<double>(path_list[i]["x"]) * scale;
    pose.position.y = static_cast<double>(path_list[i]["y"]) * scale;
    pose.position.z = static_cast<double>(path_list[i]["z"]) * scale + base_height;
    tf2::Quaternion q;
    q.setRPY(static_cast<double>(path_list[i]["roll"]),
             static_cast<double>(path_list[i]["pitch"]),
             static_cast<double>(path_list[i]["yaw"]));
    pose.orientation = tf2::toMsg(q);
    pose_vector.push_back(pose);
  }
  ROS_INFO_STREAM("Loaded path with " << pose_vector.size() << " poses");

  std::vector<ClientState> client_states;
  std::vector<LandingClientPtr> landing_clients;
  std::vector<TakeoffClientPtr> takeoff_clients;

  XmlRpc::XmlRpcValue client_list;
  private_nh.getParam("clients", client_list);
  for (int i = 0; i < client_list.size(); i++)
  {
    std::string client_name = static_cast<std::string>(client_list[i]["name"]);
    unsigned int pose_num = static_cast<int>(client_list[i]["pose"]);

    if (pose_num >= pose_vector.size())
    {
      ROS_ERROR_STREAM("Requested 0-indexed pose " << pose_num << " for client " << client_name << ", which has " <<
                       pose_vector.size() << " poses defined");
      continue;
    }

//    ros::ServiceClient enable_motors_client = nh.serviceClient<hector_uav_msgs::EnableMotors>(client_name + "/enable_motors");
//    enable_motors_client.waitForExistence();
//    hector_uav_msgs::EnableMotors srv;
//    srv.request.enable = true;
//    enable_motors_client.call(srv);
//    if(srv.response.success != true){
//      ROS_ERROR_STREAM("Could not enable motors for " << client_name);
//    }

    PoseClientPtr pose_client = boost::make_shared<PoseClient>(client_name + "/action/pose", true);
    LandingClientPtr landing_client = boost::make_shared<LandingClient>(client_name + "/action/landing", true);
    TakeoffClientPtr takeoff_client = boost::make_shared<TakeoffClient>(client_name + "/action/takeoff", true);

    ROS_WARN_STREAM("Connecting to " << client_name);
    if (!pose_client->waitForServer() ||
        !landing_client->waitForServer() ||
        !takeoff_client->waitForServer())
    {
      ROS_ERROR_STREAM("Connection to " << client_name << " failed");
      continue;
    }

    ROS_WARN_STREAM("Loaded client " << client_name << " with pose number " << pose_num);

    //Add client, -1 pose_num for 1-indexing
    client_states.push_back(std::make_pair(pose_client, pose_num));
    landing_clients.push_back(landing_client);
    takeoff_clients.push_back(takeoff_client);
  }

  for (unsigned int i = 0; i < takeoff_clients.size(); i++)
  {
    ROS_DEBUG_STREAM("Taking off");
    hector_uav_msgs::TakeoffGoal goal;
    takeoff_clients[i]->sendGoal(goal);
  }

  for (unsigned int i = 0; i < takeoff_clients.size(); i++)
  {
    takeoff_clients[i]->waitForResult();
  }

  for(int i = 0; i < pose_vector.size() * repeat + 1; i++){

    ROS_INFO_STREAM("Setting pose goal " << i + 1 << " of " << pose_vector.size() * repeat + 1);

    hector_uav_msgs::PoseGoal goal;
    goal.target_pose.header.frame_id = "world";
    goal.target_pose.header.stamp = ros::Time::now();

    for (unsigned int i = 0; i < client_states.size(); i++)
    {
      goal.target_pose.pose = pose_vector[client_states[i].second++ % pose_vector.size()];
      client_states[i].first->sendGoal(goal);
    }

    for (unsigned int i = 0; i < client_states.size(); i++)
    {
      client_states[i].first->waitForResult();
    }
  }

  for (unsigned int i = 0; i < landing_clients.size(); i++)
  {
    ROS_DEBUG_STREAM("Landing");
    hector_uav_msgs::LandingGoal goal;
    landing_clients[i]->sendGoal(goal);
  }

  for (unsigned int i = 0; i < landing_clients.size(); i++)
  {
    landing_clients[i]->waitForResult();
  }

  return 0;
}
