#include <move_robot/move_robot.h>

#include <iostream>
using std::cout;
using std::cin;
using std::endl;

#include <sstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using std::vector;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using std::string;

geometry_msgs::Pose robot_pose, old_robot_on_formation, robot_on_formation;
float grad_desc, grad_desc_mag;
bool finished_operation;

#define DIST_MIN 0.15

///////////////////////////////////
/////                         /////
///// COMUNICATION FUNCTIONS  /////
/////                         /////
///////////////////////////////////

void odom_callback(const nav_msgs::Odometry& msg){
	robot_pose = msg.pose.pose;
}

void robot_on_formation_callback(const geometry_msgs::PoseStamped& msg){
  old_robot_on_formation = robot_on_formation;
  robot_on_formation = msg.pose;
}

void grad_desc_callback(const std_msgs::Float64& msg){
  grad_desc = msg.data;
}

void grad_desc_mag_callback(const std_msgs::Float64& msg){
  grad_desc_mag = msg.data;
}

void finished_operation_callback(const std_msgs::Bool& msg){
  finished_operation = msg.data;
}

class Robot{
  public:
    char name[7] = "robot0";
    move_base_msgs::MoveBaseGoal goal, old_goal;
    int num;

    void SetupVariables();
    bool UpdateGoalPosition();
    Robot(int n){
      name[5] += n;
      num = n;
      vel_max = 0.4;

      // weights
      mi_Bi = 0.65;
      n = 0.4; 

      strcat(make_plan_topic, "/");
      strcat(make_plan_topic, name);
      strcat(make_plan_topic, "/move_base/NavfnROS/make_plan");
      test_goal_msg.request.start.header.frame_id = "map";
      test_goal_msg.request.goal.header.frame_id = "map";
      test_goal_msg.request.tolerance = 0.2;

      goal.target_pose.header.frame_id = "map";

    }

  private:
    float vel_max;
    float old_robot_orientation, old_force_z_orientation;
    float robot_orientation, force_z_orientation;
    float mi_Bi, n; // weights
    nav_msgs::GetPlan test_goal_msg;
    char make_plan_topic[37] = {};

    float Psi(float x);
    std::tuple<float, float> CalculateForce_z();

};

///////////////////////////////////
/////                         /////
/////    CENTRAL FUNCTIONS    /////
/////                         /////
///////////////////////////////////

void Robot::SetupVariables(){
  goal.target_pose.pose = robot_pose;

  old_robot_orientation = ComputeYaw(robot_pose.orientation);
  old_robot_on_formation = robot_pose;

  float z_x, z_y;
  std::tie(z_x, z_y) = CalculateForce_z();

  old_force_z_orientation = atan2(z_y, z_x);
}

float Robot::Psi(float x){
  if(x > (M_PI/2.0))  return    0.0;
  else                return cos(x);
}

std::tuple<float, float> Robot::CalculateForce_z(){
  float x, y;

  x = robot_on_formation.position.x - robot_pose.position.x;
  y = robot_on_formation.position.y - robot_pose.position.y;

  return std::make_tuple(x, y);
}

// Source: https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code
float constrainAngle(float x){
    float new_x = fmod(x + M_PI, 2.0 * M_PI);
    if (new_x < 0.0)
        new_x += 2.0 * M_PI;
    return new_x - M_PI;
}

bool Robot::UpdateGoalPosition(){

  ros::Time stamp_comum;

  old_robot_orientation = ComputeYaw(robot_pose.orientation);

  float x_k, y_k; //robot actual position

  x_k = robot_pose.position.x;
  y_k = robot_pose.position.y;

  float alfa;

  float environment, formation;

  float grad_orientation = grad_desc;
  float grad_mag = grad_desc_mag;

  robot_orientation = constrainAngle(n*old_robot_orientation + (1.0-n)*grad_orientation);

  float delta_r_x, delta_r_y;

  float z_x, z_y;
  std::tie(z_x, z_y) = CalculateForce_z();

  force_z_orientation = atan2(z_y, z_x);
  float z_mag = sqrt(pow(z_x, 2.0) + pow(z_y, 2.0));

  alfa = (z_mag < 1.0? z_mag : 1.0);
  // if(z_mag < 1.0)
  //   alfa = z_mag;
  // else
  //   alfa = 1.0;

  ///////////////x
  environment = Psi(abs(constrainAngle(old_robot_orientation - grad_orientation)))        * cos(robot_orientation);
  formation =   Psi(abs(constrainAngle(  force_z_orientation - old_force_z_orientation))) * cos(force_z_orientation);
  delta_r_x =  vel_max * (mi_Bi * environment + (1.0-mi_Bi) * alfa * formation);
    
  //////////////y
  environment = Psi(abs(constrainAngle(old_robot_orientation - grad_orientation)))        * sin(robot_orientation);
  formation =   Psi(abs(constrainAngle(  force_z_orientation - old_force_z_orientation))) * sin(force_z_orientation);
  delta_r_y =  vel_max * (mi_Bi * environment + (1.0-mi_Bi) * alfa * formation);

  old_force_z_orientation = force_z_orientation;

  goal.target_pose.pose.position.x = x_k + delta_r_x;
  goal.target_pose.pose.position.y = y_k + delta_r_y;
  goal.target_pose.pose.position.z = 0.0;

  goal.target_pose.pose.orientation = computeQuaternionFromYaw(robot_orientation);

  stamp_comum = ros::Time::now();

	goal.target_pose.header.stamp = stamp_comum;

  test_goal_msg.request.start.pose = robot_pose;
  test_goal_msg.request.start.header.stamp = stamp_comum;
  test_goal_msg.request.goal = goal.target_pose;

  bool resul = ros::service::call(make_plan_topic, test_goal_msg);

  return (test_goal_msg.response.plan.poses.size() > 0);
}

///////////////////////////////////
/////                         /////
/////      MAIN FUNCTION      /////
/////                         /////
///////////////////////////////////

int main(int argc, char** argv){
	// Initialize
	ros::init(argc, argv, "move_robot", ros::init_options::AnonymousName);

  ros::NodeHandle n("~");

  int i;

  std::string param_i;
  if (n.searchParam("robot_num", param_i)){
    n.getParam(param_i, i);
  }else{
    ROS_WARN("No param 'robot_num' found in an upward search");
    return 0;
  }

  Robot robot(i);

  float dist_robot_goal=DIST_MIN + 1.0;

  auto node_name = ros::this_node::getName();
  ROS_WARN("ROBOT: %s on the node %s", robot.name, node_name.c_str());

  // To use the clear_costmaps service
  std_srvs::Empty emptymsg;
  char clear_costmaps_topic[33]={};
  strcat(clear_costmaps_topic, "/");
  strcat(clear_costmaps_topic, robot.name);
  strcat(clear_costmaps_topic, "/move_base/clear_costmaps");

  // SET SUBSCRIBERS
  ros::Subscriber odom_sub, map_sub, robot_on_formation_sub, robot_grad_sub, robot_grad_mag_sub, finished_operation_sub;

  char odom_topic[1+6+1+4+1] = {};
  strcat(odom_topic, "/");
  strcat(odom_topic, robot.name);
  strcat(odom_topic, "/odom");
  odom_sub = n.subscribe(odom_topic, 1, odom_callback);
 
  char move_base_topic[1+6+1+9+1] = {};
  strcat(move_base_topic, "/");
  strcat(move_base_topic, robot.name);
  strcat(move_base_topic, "/move_base");
	MoveBaseClient ac(move_base_topic, true);

  char robot_on_formation_topic[1+5+1+14] = {};
  strcat(robot_on_formation_topic, "/");
  strcat(robot_on_formation_topic, robot.name);
  strcat(robot_on_formation_topic, "_on_formation");
	robot_on_formation_sub = n.subscribe(robot_on_formation_topic, 1, robot_on_formation_callback);

  char robot_grad_topic[1+5+1+11] = {};
  strcat(robot_grad_topic, "/");
  strcat(robot_grad_topic, robot.name);
  strcat(robot_grad_topic, "_grad_desc");
  robot_grad_sub = n.subscribe(robot_grad_topic, 1, grad_desc_callback);
  grad_desc = 0.0;
  strcat(robot_grad_topic, "_mag");
  robot_grad_mag_sub = n.subscribe(robot_grad_topic, 1, grad_desc_mag_callback);
  grad_desc_mag = 0.0;

  finished_operation_sub = n.subscribe("/finished_operation", 1, finished_operation_callback);
  finished_operation = false;

  ros::spinOnce();

	// Wait 5 sec for move base action server to come up
	while(!ac.waitForServer (ros::Duration(5.0))){
		ROS_WARN("Waiting for the move_base action server: %s :for %s to come up", move_base_topic, robot.name);
    if(!ros::ok()){
      return 0;
    }
  }

	ROS_WARN("OK move_base action server: %s :for %s", move_base_topic, robot.name);

  ros::spinOnce();
	  
	ros::Duration(3.0).sleep();
  ros::spinOnce();

  // Start algoritm

  robot.SetupVariables();

  ros::Duration(3.0).sleep();

  int cont = 0;

	while (ros::ok() && !finished_operation){
		ros::spinOnce();
    if(robot.UpdateGoalPosition()){
      ac.sendGoal (robot.goal);
    }
    cont++;

    if(cont%2 == 0)
      ros::service::call(clear_costmaps_topic, emptymsg);
    if(cont >= 1000000000) cont = 0;

    ros::Duration(1.0).sleep();
  }
  
  ROS_WARN("ROBOT: %s finished the operation", robot.name);
  return 0;
}
