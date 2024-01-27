#include "ros/ros.h"
#include <std_msgs/String.h>
#include <custom_msgs/Obstacles.h>
#include <custom_msgs/Form.h>
#include <nlohmann/json.hpp>
#include "SemanticPlanner.h"

#include "jsoncpp/json/json.h"
#include <fstream>

// Return object list
custom_msgs::Obstacles parseMapConfig(const std::string &configPath)
{
    custom_msgs::Obstacles objs;
    std::ifstream ifs(configPath);
    Json::Reader reader;
    Json::Value root;
    // if (!reader.parse(ifs, root)) {
    //     std::cout << "Failed to parse " << configPath << std::endl;
    //     return;
    // }

    Json::Value obstacles_json = root["obstacles"];
    for (int i = 0; i < obstacles_json.size(); ++i) {
        // Obstacle obs;
        custom_msgs::Form obs;
        Json::Value obstacle_json = obstacles_json[i];
        // obs.type = obstacle_json["type"].asInt();
        Json::Value points_json = obstacle_json["vertices"];
        for (int j = 0; j < points_json.size(); ++j) {
            geometry_msgs::Point p;
            Json::Value point_json = points_json[j];
            p.x = point_json["x"].asInt();
            p.y = point_json["y"].asInt();
            obs.form.push_back(p);
        }
        objs.list.push_back(obs);
    }

    // TODO: parse json file

    return objs;
}

// Return mission list
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

    return mission;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_object_layer");
    // Create a handle to this process node
    ros::NodeHandle n;
    // Publisher is chatter_pub publishing to topic /object_costmap_layer/obsctacles with queue_size 1000
    ros::Publisher chatter_pub = n.advertise<custom_msgs::Obstacles>("/object_costmap_layer/obsctacles_temp", 1000);
    std::string param_file_path;
    std::string object_layer_config;
    std::string mission_config;

    n.getParam("/add_object_layer/object_layer", param_file_path);
    n.getParam("/add_object_layer/map", object_layer_config);
    n.getParam("/add_object_layer/mission", mission_config);

    ROS_INFO("param_file_path_ is %s", param_file_path.c_str());
    ROS_INFO("object_layer_config is %s", object_layer_config.c_str());
    ROS_INFO("mission_config is %s", mission_config.c_str());

    custom_msgs::Obstacles objs = parseMapConfig(object_layer_config);
    std::vector<WayPoint> mission = parseMissionConfig(mission_config);

    int  count = 0;
    ros::Rate rate(1);
    while (ros::ok())
    {
        // create message msg with date type is string
        std_msgs::String msg;

        // point experiment
        float point_[4][3] = {{0.22,-0.6,0},
                         {0.22,-1.8,0},
                         {-3.6,-1.7,0},
                         {-3.6,-0.35,0}};


        int number_circle = 8;
        int number_objs = 1;

        custom_msgs::Obstacles temp;
        // add object layer
        for (int i = 0; i < number_objs; i++)
        {
            custom_msgs::Form obs;
            for (int n = 0 ; n < 4; n++) {
                geometry_msgs::Point p;
                 p.x = point_[n+i*4][0];
                 p.y = point_[n+i*4][1];
                 p.z = point_[n+i*4][2];
                obs.form.push_back(p);
            }
            temp.list.push_back(obs);
        }
        temp = objs;
        chatter_pub.publish(temp);
        
        ros::spinOnce();
        ++count;
        rate.sleep();
    }
    return 0;
}



