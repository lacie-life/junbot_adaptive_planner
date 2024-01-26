#include "ros/ros.h"
#include <std_msgs/String.h>
#include <custom_msgs/Obstacles.h>

#include "SemanticPlanner.h"

#include "jsoncpp/json/json.h"
#include <fstream>

// Return object list
custom_msgs::Obstacles parseMapConfig(const std::string &configPath)
{
    custom_msgs::Obstacles objs;

    std::ifstream ifs(configPath + "/map.json");
    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(ifs, root)) {
        std::cout << "Failed to parse " << configPath << std::endl;
        return objs;
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
    ros::init(argc, argv, "add_layer");
    // Create a handle to this process node
    ros::NodeHandle n;
    // Publisher is chatter_pub publishing to topic /object_costmap_layer/obsctacles with queue_size 1000
    ros::Publisher chatter_pub = n.advertise<custom_msgs::Obstacles>("/object_costmap_layer/obsctacles_temp", 1000);

    std::string param_file_path;
    std::string object_layer_config;
    std::string mission_config;
    n.getParam("object_layer", param_file_path);
    object_layer_config = param_file_path['map'];
    mission_config = param_file_path['mission'];

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

        chatter_pub.publish(temp);
        
        ros::spinOnce();
        ++count;
        rate.sleep();
    }
    return 0;
}



