// Source: https://answers.ros.org/question/377450/costmap2dros-object-is-not-correctly-created/

// Source: https://answers.ros.org/question/228253/generating-a-2d-costmap-from-a-mapyaml-file/

// Source: http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
// #include <dwa_local_planner/dwa_planner_ros.h>
// #include <dwa_local_planner/dwa_planner.h>

geometry_msgs::PoseStamped start, goal;

// void start_callback(const geometry_msgs::PoseStamped& msg){
// 	start = msg;
//   // map_recevied = msg;
// }

void goal_callback(const geometry_msgs::PoseStamped& msg){
	goal = msg;
  // map_recevied = msg;
	// ROS_INFO("MAPA seq %d", map_recevied.header.seq);//, map_recevied.info.width);
}

int main (int argc, char** argv)
{
  ros::init(argc, argv,"ros_global_planners_experiment_node");

  ros::NodeHandle n("~");

  // ros::Subscriber start_sub = n.subscribe("clicked_point", 1, start_callback);
  // ros::Subscriber goal_sub = n.subscribe("clicked_point", 1, goal_callback);

  start.header.frame_id = "map";
  start.header.stamp = ros::Time::now();
  start.pose.position.x = 0;
  start.pose.position.y = 0;
  start.pose.position.z = 0;
  start.pose.orientation.w = 1;
  start.pose.orientation.x = 0;
  start.pose.orientation.y = 0;
  start.pose.orientation.z = 0;

  goal = start;
  goal.pose.position.x = 1;

//   tf::TransformListener tf(ros::Duration(10));
//   costmap_2d::Costmap2DROS willow_garage_costmap( "willowgarage_costmap", tf);

  tf2_ros::Buffer tfBuffer(ros::Duration(10));
  tf2_ros::TransformListener tfListener(tfBuffer);
  costmap_2d::Costmap2DROS test_costmap("global_costmap", tfBuffer);

  ROS_WARN("Costmap2d node started ...");

  navfn::NavfnROS navfn;
  navfn.initialize("my_navfn_planner", &test_costmap);

  std::vector<geometry_msgs::PoseStamped> plan;

  int tam;

  while (ros::ok()){
    ros::spinOnce();
    if (navfn.makePlan(start, goal, plan)){
      tam= plan.size();
      ROS_WARN("FIND PLAN OF %d", tam);
      // pub at /global_planners_test_node/my_navfn_planner/plan
    }else{
      ROS_WARN("NOPE");
    }
  }
  
  return 0;
}