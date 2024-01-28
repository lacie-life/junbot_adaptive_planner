#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include "SemanticPlanner.h"
#include "jsoncpp/json/json.h"
#include <fstream>


move_base_msgs::MoveBaseGoal tempGoal;
ros::Publisher pub;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::vector<WayPoint> parseMissionConfig(const std::string &configPath)
{
    std::vector<WayPoint> mission;

    std::ifstream ifs(configPath);
    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(ifs, root)) {
        std::cout << "Failed to parse " << configPath << std::endl;
        return mission;
    }
    // TODO: parse json file
    Json::Value mission_waypoint = root["vehicle_states"];
    ROS_INFO("mission_waypoint size is %d", mission_waypoint.size());

    for (int i = 0; i < mission_waypoint.size(); ++i) {
        WayPoint wp;
        Json::Value waypoint_json = mission_waypoint[i];
        wp.type = waypoint_json["type"].asString();

        Json::Value pose = waypoint_json["rear_wheel_position"];

        wp.pose.x = pose[0].asDouble();
        wp.pose.y = pose[1].asDouble(); 
        wp.pose.yaw = pose[2].asDouble();

        mission.push_back(wp);
    }

    return mission;
}

int main(int argc, char **argv)
{
    actionlib_msgs::GoalID tempCancel;
    ros::init(argc, argv, "mission_publisher");
    MoveBaseClient ac("move_base", true);
    ros::NodeHandle n;
    std::string mission_config;

    n.getParam("/mission_control/mission_path", mission_config);
    ROS_INFO("object_layer_config is %s", mission_config.c_str());
    std::vector<WayPoint> mission = parseMissionConfig (mission_config);
    // ros::Publisher  cancel = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1000);

    // pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000);
    // ros::Rate loop_rate(10);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    tempGoal.target_pose.pose.position.x = mission.at(1).pose.x;
    tempGoal.target_pose.pose.position.y = mission.at(1).pose.y;
    tempGoal.target_pose.pose.orientation.w = 1.0;
    tempGoal.target_pose.header.frame_id = "base_link";
    tempGoal.target_pose.header.stamp = ros::Time::now();

    ac.sendGoal(tempGoal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved 1 meter forward");
    else
        ROS_INFO("The base failed to move forward 1 meter for some reason");
    // Publish mission
    // while (ros::ok())
    // {

    //     ROS_INFO("GOALLLLLLLLLL");
    //     pub.publish(tempGoal);
    //     break;
    //     // int c = getchar();   // call your non-blocking input function
    //     // if (c == 'a')
    //     // {
    //     //     tempCancel.stamp = {};
    //     //     tempCancel.id = {};
    //     //     cancel.publish(tempCancel);
    //     // }
    //     // else if (c == 'b')
    //     // {
    //     //     pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000);
    //     //     pub.publish(tempGoal);
    //     // }
    //     ros::Rate loop_rate(10);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
  return 0;
}