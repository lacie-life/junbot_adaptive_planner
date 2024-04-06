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
#include <stack>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "concavehull.hpp"

double calculateDistance (double Xa, double Ya, double Xb, double Yb);
// TODO: Subscribe map infor

struct Point
{
    double x;
    double y;
    double z;
};

struct field_value
{
    double x;
    double y;
    double value;
};


ros::Subscriber subPlan;
ros::Subscriber subObj;
ros::Publisher pubObj;
geometry_msgs::PoseWithCovariance amcl_pose;
nav_msgs::Path path;
ros::Publisher pub;
custom_msgs::Obstacles object;
std::mutex m;
double vel_robot[2];
std::vector<field_value> res;
field_value p0;

int compare(const void* a, const void* b)
{
	const int* x = (int*) a;
	const int* y = (int*) b;

	if (*x > *y)
		return 1;
	else if (*x < *y)
		return -1;

	return 0;
}

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

std::vector<Point> findPointsOnRectangleEdges(Point A, Point B, Point C, Point D) {
    std::vector<Point> points;

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

double dot_product(double vector1[], double vector2[]) {
    return vector1[0] * vector2[0] + vector1[1] * vector2[1];
}

double norm(double vector[]) {
    return sqrt(vector[0] * vector[0] + vector[1] * vector[1]);
}

// A utility function to swap two points
void swap_point(field_value &p1, field_value &p2)
{
    field_value temp = p1;
    p1 = p2;
    p2 = temp;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(field_value p, field_value q, field_value r)
{
    int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
 
    if (val == 0) return 0;  // collinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}

// A utility function to find next to top in a stack
field_value nextToTop(std::stack<field_value> &S)
{
    field_value p = S.top();
    S.pop();
    field_value res1 = S.top();
    S.push(p);
    return res1;
}

/* Calculate value for field of a center point*/
void calculate_field(Point center_point){
    double sx = 1;
    double sy = 1;
    double rel = 0.1; // resolution
    double distance_center_robot = sqrt(((amcl_pose.pose.position.x - center_point.x)*(amcl_pose.pose.position.x - center_point.x))
    +((amcl_pose.pose.position.y - center_point.y)*(amcl_pose.pose.position.y - center_point.y))); // distance between robot and center point
    for (double x = center_point.x+distance_center_robot; x < center_point.x-distance_center_robot; x = x - rel)
    {
        for (double y = center_point.y+distance_center_robot; x < center_point.y-distance_center_robot; y = y - rel)
        {
            field_value point_value;
            double value_temp = 0;
            double distance = sqrt(((amcl_pose.pose.position.x - center_point.x) * (amcl_pose.pose.position.x - center_point.x) / (distance_center_robot * distance_center_robot))
                + ((amcl_pose.pose.position.y - center_point.y) * (amcl_pose.pose.position.y - center_point.y) / (distance_center_robot * distance_center_robot)));

            if (distance != 0) {
                double vector1[] = { x - center_point.x, y - center_point.y };
                double vector2[] = {1, 2};//vel_x, vel_y }; // velocity of obstacle
                vector2[0] += vel_robot[0];
                vector2[1] += vel_robot[1];
                double theta = acos(dot_product(vector1, vector2) / (norm(vector1) * norm(vector2)));
                // res_temp = epsilon*norm(vector2)(1 + cos(theta)) * pow((1 - 1 / distance), 2);
                value_temp = (200 + 200 * cos(theta)) * pow((1 - 1 / distance), 2);
            }
            else {
                value_temp = 0;
            }
            point_value.x = x;
            point_value.y = y;
            point_value.value = value_temp;
            res.push_back(point_value);
        }
    } 
}

// Prints convex hull of a set of n points.
std::vector<field_value> convexHull()
{
    int n = res.size();
    // Find the bottommost point
    int ymin = res.at(0).y;
    int min = 0;
    for (int i = 1; i < n; i++)
    {
        int y = res.at(i).y;
        // Pick the bottom-most or choose the left
        // most point in case of tie
        if ((y < ymin) || (ymin == y && res.at(i).x < res.at(min).x))
        {
            ymin = res.at(i).y;
            min = i;
        }
    }
    
    // Place the bottom-most point at first position
    swap_point(res.at(0), res.at(min));
 
    // Sort n-1 points with respect to the first point.
    // A point p1 comes before p2 in sorted output if p2
    // has larger polar angle (in counterclockwise
    // direction) than p1
    p0 = res.at(0);
    qsort(&res.at(1), n-1, res.size(), compare);
    
    // If two or more points make same angle with p0,
    // Remove all but the one that is farthest from p0
    // Remember that, in above sorting, our criteria was
    // to keep the farthest point at the end when more than
    // one points have same angle.
    int m = 1; // Initialize size of modified array
    for (int i=1; i<n; i++)
    {
        // Keep removing i while angle of i and i+1 is same
        // with respect to p0
        while (i < n-1 && orientation(p0, res.at(i),res.at(i+1)) == 0){
            i++;
        }
        res.at(m) = res.at(i);
        m++;  // Update size of modified array
    }
    
    // If modified array of points has less than 3 points,
    // convex hull is not possible
    if (m < 3) return std::vector<field_value>();
 
    // Create an empty stack and push first three points
    // to it.
    std::stack<field_value> S;
    S.push(res.at(0));
    S.push(res.at(1));
    S.push(res.at(2));
    
    // Process remaining n-3 points
    for (int i = 3; i < m; i++)
    {
        // Keep removing top while the angle formed by
        // points next-to-top, top, and points[i] makes
        // a non-left turn
        while (S.size()>1 && orientation(nextToTop(S), S.top(), res.at(i)) != 2)
            S.pop();
        S.push(res.at(i));
    }
    std::vector<field_value> hull;
    while (!S.empty()) {
        hull.push_back(S.top());
        S.pop();
    }
    return hull;
    
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
    subObj = n.subscribe("/object_costmap_layer/obstacles_temp", 1000, objectCallback);
    pubObj = n.advertise<custom_msgs::Obstacles>("/object_costmap_layer/obstacles", 1000);

    double coner_temp[4][3] = {{0.22,-0.6,0},
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

    std::vector<Point> center_point = findPointsOnRectangleEdges(coner.at(0), coner.at(1), coner.at(2),coner.at(3));
    for (int i = 0; i < center_point.size(); i++)
    {
        calculate_field(center_point.at(i));
    }
    for (int i = 0; i < res.size(); i++)
    {
        if (res.at(i).value < 0 || res.at(i).value > 99)
        {
            res.erase(res.begin()+i);
        }
    }
    std::vector<field_value> bound = convexHull ();
                 
    while (ros::ok())
    {
        m.lock();
        custom_msgs::Obstacles objectNew;
        for (int i = 0; i < 1; ++i) {// number object
            for (int j = 0; j < bound.size(); ++j) { // number point of object
                geometry_msgs::Point p;
                p.x = bound.at(j).x;
                p.y = bound.at(j).y;
                p.z = 0;
                objectNew.list[i].form.push_back(p);
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
