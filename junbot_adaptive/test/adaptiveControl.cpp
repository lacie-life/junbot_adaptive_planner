//
// Created by lacie on 17/02/2023.
//
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <custom_msgs/Obstacles.h>
#include <mutex>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"

double calculateDistance (double Xa, double Ya, double Xb, double Yb);
// TODO: Subscribe map infor

struct Point
{
    float x;
    float y;
    float z;
};

struct field_value
{
    float x;
    float y;
    float value;
};

ros::Subscriber subPlan;
ros::Subscriber subObj;
ros::Publisher pubObj;
geometry_msgs::PoseWithCovariance amcl_pose;
nav_msgs::Path path;
ros::Publisher pub;
custom_msgs::Obstacles object;
std::mutex m;
float vel_robot[2];
std::vector<field_value> res;

void globalPlanCallback(nav_msgs::Path::ConstPtr tempPath)
{
    m.lock();
    for (int i = 0; i < tempPath->poses.size(); i++)
    {
        path.poses.push_back(tempPath->poses[i]);
    }
    m.unlock();
//    path = tempPath;
}

void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg_amcl)
{
    amcl_pose = msg_amcl->pose;
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    vel_robot[0]= msg->linear.x;
    vel_robot[1] = msg->linear.y;
}

void objectCallback(custom_msgs::Obstacles::ConstPtr objTemp)
{
    m.lock();
    object.list = objTemp->list;
    pubObj.publish(object);
    ROS_INFO("Object Callback");
    for(int i = 0; i < 4; i++)
    {
        std::cout << object.list.at(0).form.at(i).x << "\n";
        std::cout << object.list.at(0).form.at(i).y << "\n";
        std::cout << object.list.at(0).form.at(i).z << "\n";
    }
    m.unlock();
}

double distanceBetweenPoints(Point p1, Point p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

vector<Point> findPointsOnRectangleEdges(Point A, Point B, Point C, Point D) {
    vector<Point> points;

    int num_points_AB = distanceBetweenPoints(A, B) / 0.1;
    int num_points_BC = distanceBetweenPoints(B, C) / 0.1;
    int num_points_CD = distanceBetweenPoints(C, D) / 0.1;
    int num_points_DA = distanceBetweenPoints(D, A) / 0.1;

    double increment_AB_x = (B.x - A.x) / num_points_AB;
    double increment_AB_y = (B.y - A.y) / num_points_AB;
    for (int i = 0; i <= num_points_AB; ++i) {
        points.push_back({A.x + i * increment_AB_x, A.y + i * increment_AB_y});
    }

    double increment_BC_x = (C.x - B.x) / num_points_BC;
    double increment_BC_y = (C.y - B.y) / num_points_BC;
    for (int i = 0; i <= num_points_BC; ++i) {
        points.push_back({B.x + i * increment_BC_x, B.y + i * increment_BC_y});
    }

    double increment_CD_x = (D.x - C.x) / num_points_CD;
    double increment_CD_y = (D.y - C.y) / num_points_CD;
    for (int i = 0; i <= num_points_CD; ++i) {
        points.push_back({C.x + i * increment_CD_x, C.y + i * increment_CD_y});
    }

    double increment_DA_x = (A.x - D.x) / num_points_DA;
    double increment_DA_y = (A.y - D.y) / num_points_DA;
    for (int i = 0; i <= num_points_DA; ++i) {
        points.push_back({D.x + i * increment_DA_x, D.y + i * increment_DA_y});
    }
    return points;
}

double dot_product(std::vector<double>& vector1, std::vector<double>& vector2) {
    return vector1[0] * vector2[0] + vector1[1] * vector2[1];
}

double norm(std::vector<double>& vector) {
    return sqrt(vector[0] * vector[0] + vector[1] * vector[1]);
}

void calculate_field(Point center_point){
    float sx = 1;
    float sy = 1;
    float rel = 0.1;
    distance_center_robot = sqrt(((amcl_pose.pose.pose.position.x - center_point.x)(amcl_pose.pose.pose.position.x - center_point.x))
    +((amcl_pose.pose.pose.position.y - center_point.y)(amcl_pose.pose.pose.position.y - center_point.y)));
    for (float x = center_point.x+distance_center_robot; x < center_point.x-distance_center_robot; x = x - rel)
    {
        for (float y = center_point.y+distance_center_robot; x < center_point.y-distance_center_robot; y = y - rel)
        {

            float distance = sqrt(((amcl_pose.pose.pose.position.x - center_point.x)(amcl_pose.pose.pose.position.x - center_point.x)/(distance_center_robot*distance_center_robot))
            +((amcl_pose.pose.pose.position.y - center_point.y)(amcl_pose.pose.pose.position.y - center_point.y)/(distance_center_robot*distance_center_robot)));
            
            if (d != 0){
                float vector1 = [x-center_point.x y-center_point.x];
                float vector2 = [vel_x vel_y]; // velocity of obtacle
                vector2 = vector2 + vel_robot;
                double theta = acos(dot_product(vector1, vector2) / (norm(vector1) * norm(vector2)));
                // res_temp = epsilon*norm(vector2)(1 + cos(theta)) * pow((1 - 1 / distance), 2);
                res_temp = (200 + 200 * cos(theta)) * pow((1 - 1 / distance), 2);
            }
            else{
                res_temp= 0;
            }
            field_value field_temp;
            field_temp.x = x;
            field_temp.y = y;
            field_temp.value = res_temp;
            res.push_back(field_temp);
        }
    }
    
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    subPlan = n.subscribe("/move_base/DWAPlannerROS/global_plan", 10000, globalPlanCallback);
    ros::Subscriber sub_amcl = n.subscribe("/amcl_pose", 1000, poseAMCLCallback);
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmdVelCallback);
    subObj = n.subscribe("/object_costmap_layer/obstacles_temp", 10000, objectCallback);
    pubObj = n.advertise<custom_msgs::Obstacles>("/object_costmap_layer/obstacles", 1000);

    float coner_temp[4][3] = {{0.22,-0.6,0},
                                {0.22,-1.8,0},
                                {-3.6,-1.7,0},
                                {-3.6,-0.35,0}};
    // coner experiment
    ros::Rate rate(1);
    std::vector<Point> coner;
    for (int i = 0; i < 4; i++)
    {
        Point temp;
        temp.x = coner_temp[i][0];
        temp.y = coner_temp[i][1];
        coner.push_back(temp);

    }
    std::vector<Point> center_point = findPointsOnLine(coner.at(0), coner.at(1), coner.at(2),coner.at(3));
    for (int i = 0; i < center_point.size(); i++)
    {
        calculate_field(center_point.at(i));
    }
                 
    while (ros::ok())
    {
        m.lock();
        double distance = 99;
        custom_msgs::Obstacles objectNew = object;
        for (int i = 0; i < 1; ++i) {
            for (int j = 0; j < 4; ++j) {
                std::vector<geometry_msgs::Point> waypointError;
                custom_msgs::Form temp;
                for (int k = 0; k < path.poses.size(); ++k) {
                    distance = calculateDistance(object.list[i].form[j].x, object.list[i].form[j].y,path.poses[k].pose.position.x, path.poses[k].pose.position.y);
                    if (distance <= 0.03) {
                        geometry_msgs::Point p;
                        p.x = path.poses[k].pose.position.x;
                        p.y = path.poses[k].pose.position.y;
                        p.z = path.poses[k].pose.position.z;
                        waypointError.push_back(p);
                    }
                }
                
                // if (waypointError.size()>1)
                // {
                //     geometry_msgs::Point _begin = waypointError.at(0);
                //     geometry_msgs::Point _end = waypointError.at(waypointError.size() - 1);
                //     temp.form.push_back(_begin);
                //     temp.form.push_back(_end);
                //     temp.form.push_back(object.list[i].form[j]);
                //     temp.id = "zone";
                //     objectNew.list.push_back(temp);
                // }
            }
        }
        for (int j = 0; j < 4; ++j) {
            std::vector<geometry_msgs::Point> waypointError;
            custom_msgs::Form temp;
            for (int k = 0; k < path.poses.size(); ++k) {
                distance = calculateDistance(coner[j][0],coner[j][1],path.poses[k].pose.position.x, path.poses[k].pose.position.y);                    
                if (distance <= 0.3) {
                    geometry_msgs::Point p;
                    p.x = path.poses[k].pose.position.x;
                    p.y = path.poses[k].pose.position.y;
                    p.z = path.poses[k].pose.position.z;
                    waypointError.push_back(p);
                }
            }
            if (waypointError.size()>1)
            {
                geometry_msgs::Point _begin = waypointError.at(0);
                geometry_msgs::Point _end = waypointError.at(waypointError.size() - 1);
                geometry_msgs::Point coner_;
                coner_.x = coner[j][0];
                coner_.y = coner[j][1];
                coner_.z = coner[j][2];
                temp.form.push_back(_begin);
                temp.form.push_back(_end);
                temp.form.push_back(coner_);
                temp.id = "zone";
                objectNew.list.push_back(temp);
            }
        }
        pubObj.publish(objectNew);
        m.unlock();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

double calculateDistance (double Xa, double Ya, double Xb, double Yb)
{
    return sqrt((Xb-Xa)*(Xb-Xa)+(Yb-Ya)*(Yb-Ya));
}
