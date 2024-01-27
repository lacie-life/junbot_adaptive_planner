#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include "SemanticPlanner.h"
#include "jsoncpp/json/json.h"
#include <fstream>


move_base_msgs::MoveBaseActionGoal tempGoal;
ros::Publisher pub;

std::vector<WayPoint> parseMissionConfig(const std::string &configPath)
{
    std::vector<WayPoint> mission;

    std::ifstream ifs(configPath + "/mission.json");
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
    ros::NodeHandle n;
    std::string mission_config;

    n.getParam("/add_object_layer/mission", mission_config);

    ros::Publisher  cancel = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1000);
    
    ros::Rate loop_rate(10);

    // Publish mission
    while (ros::ok())
    {
        int c = getchar();   // call your non-blocking input function
        if (c == 'a')
        {
            tempCancel.stamp = {};
            tempCancel.id = {};
            cancel.publish(tempCancel);
        }
        else if (c == 'b')
        {
            pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000);
            pub.publish(tempGoal);
        }
        ros::Rate loop_rate(10);
        ros::spinOnce();
        loop_rate.sleep();
    }
  return 0;
}