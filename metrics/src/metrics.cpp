#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

//////////para mover a formacao
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stack>
#include <vector>
#include <cmath>

using std::cout;
using std::cin;
using std::endl;

#include <sstream>

using std::string;
using std::vector;

#define TIME_STEP 1.0 ////seconds
#define NUM_ROBOTS 4

/*TOPICS TO BE MONITORED
/formation_center -> position and orientation
/robotX/odom -> position and orientation
/robotX_on_formation -> position and orientation (ideal position)
/test_point -> intermediate goal, position and orientation

It start with /init_operation == true
It calculates time too, ends with /finished_operation == true
*/

#include <iostream>
#include <chrono>
#include <thread>

#include <ctime>   // localtime
#include <iomanip> // put_time

/// Source: https://stackoverflow.com/questions/17223096/outputting-date-and-time-in-c-using-stdchrono
std::string return_current_time_and_date(){
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%X");
    return ss.str();
}

bool init_operation = false, finished_operation = false;
geometry_msgs::Pose formation_center, intermediate_goal, robots_odom[NUM_ROBOTS+1], robots_ideal[NUM_ROBOTS+1];

void init_operation_callback(const std_msgs::Bool& msg){
	init_operation = msg.data;
}

void finished_operation_callback(const std_msgs::Bool& msg){
	finished_operation = msg.data;
}

void formation_center_callback(const geometry_msgs::PoseStamped& msg){
	formation_center = msg.pose;
}

void intermediate_goal_callback(const geometry_msgs::PoseStamped& msg){
	intermediate_goal = msg.pose;
}

void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg, int n_robot){
	robots_odom[n_robot] = msg->pose.pose;
}

void robot_ideal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg, int n_robot){
	robots_ideal[n_robot] = msg->pose;
}

int main(int argc, char** argv){

	using namespace std::literals::chrono_literals;

    // Initialize
    ros::init(argc, argv, "metrics");
    ros::NodeHandle n("~");

    std::string param_dir, dir;
    if (n.searchParam("dir_arq", param_dir)){
        n.getParam(param_dir, dir);
    }else{
        ROS_WARN("No param 'dir_arq' found in an upward search");
        return 0;
    }

    // SET SUBSCRIBERS
    ros::Subscriber init_operation_sub, finished_operation_sub, formation_center_sub, intermediate_goal_sub;
    ros::Subscriber robots_odom_sub[NUM_ROBOTS+1], robots_ideal_sub[NUM_ROBOTS+1];

    init_operation_sub = n.subscribe("/init_operation", 1, init_operation_callback);
    finished_operation_sub = n.subscribe("/finished_operation", 1, finished_operation_callback);
    formation_center_sub = n.subscribe("/formation_center", 1, formation_center_callback);
    intermediate_goal_sub = n.subscribe("/test_point", 1, intermediate_goal_callback);

    for(int it = 1; it <= NUM_ROBOTS; it++){
        char odom_topic[1+6+1+4+1] = {};
        strcat(odom_topic, "/robot");
        strcat(odom_topic, std::to_string(it).c_str());
        strcat(odom_topic, "/odom");
        robots_odom_sub[it]  = n.subscribe<nav_msgs::Odometry>(odom_topic, 1, boost::bind(&robot_odom_callback, _1, it));

        char robot_on_formation_topic[1+5+1+14] = {};
        strcat(robot_on_formation_topic, "/robot");
        strcat(robot_on_formation_topic, std::to_string(it).c_str());
        strcat(robot_on_formation_topic, "_on_formation");
        robots_ideal_sub[it]= n.subscribe<geometry_msgs::PoseStamped>(robot_on_formation_topic, 1, boost::bind(&robot_ideal_callback, _1, it));
    }

    do{
        ROS_INFO("NOT STARTED");
        ros::Duration(TIME_STEP).sleep();
        ros::spinOnce();
    }while(!init_operation && ros::ok());
    
    ROS_WARN("STARTED");
   
    char file_name[50] = {};
    strcat(file_name, dir.c_str());
    strcat(file_name, "/test_");
    strcat(file_name, return_current_time_and_date().c_str());
    strcat(file_name, ".csv");
    std::ofstream log;
    ROS_WARN("log at %s", file_name);

    log.open (file_name);

    log << "TIME_STEP, " << TIME_STEP << ", seconds interval between records\n formation_center.position.x, formation_center.position.y, intermediate_goal.position.x, intermediate_goal.position.y, ";

    for(int it = 1; it <= NUM_ROBOTS; it++){
        log << "robot" << it << "_odom.position.x, robot" << it << "_odom.position.y, ";
        log << "robot" << it << "_ideal.position.x, robot" << it << "_ideal.position.y, ";
    }
    log << "\n";

	auto start = std::chrono::high_resolution_clock::now(); //this gives us the current time

    do{
        ros::Duration(TIME_STEP).sleep();
        ros::spinOnce();

        log << formation_center.position.x <<  ", " << formation_center.position.y << ", " << intermediate_goal.position.x <<  ", " << intermediate_goal.position.y <<  ", " ;
   
        for(int it = 1; it <= NUM_ROBOTS; it++){
            log << robots_odom[it].position.x << ", " << robots_odom[it].position.y << ", ";
            log << robots_ideal[it].position.x << ", " << robots_ideal[it].position.y << ", ";
        }
        log << "\n";
        
    }while(!finished_operation && ros::ok());

    ROS_WARN("ENDED");

	auto end = std::chrono::high_resolution_clock::now();

	std::chrono::duration<float> duration = end - start; //we are computing the time difference
	log << "TOTAL TIME, " << duration.count() << ", seconds " << std::endl;

    log.close();

    return 0;
}