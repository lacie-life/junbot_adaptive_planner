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
    if (!reader.parse(ifs, root)) {
        std::cout << "Failed to parse " << configPath << std::endl;
        return objs;
    }

    Json::Value obstacles_json = root["obstacles"];

    ROS_INFO("obstacles_json size is %d", obstacles_json.size());
    
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
    return objs;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_object_layer");
    // Create a handle to this process node
    ros::NodeHandle n;
    // Publisher is chatter_pub publishing to topic /object_costmap_layer/obsctacles with queue_size 1000
    ros::Publisher chatter_pub = n.advertise<custom_msgs::Obstacles>("/object_costmap_layer/obsctacles_temp", 1000);
    std::string object_layer_config;
    
    n.getParam("/add_object_layer/map", object_layer_config);
    ROS_INFO("object_layer_config is %s", object_layer_config.c_str());

    custom_msgs::Obstacles objs = parseMapConfig(object_layer_config);

    ROS_INFO("objs size is %d", objs.list.size());

    int  count = 0;
    ros::Rate rate(1);
    while (ros::ok())
    {
        ROS_INFO("Publishing object layer");
        chatter_pub.publish(objs);
        
        ros::spinOnce();
        ++count;
        rate.sleep();
    }
    return 0;
}



