#include <central_control/central_control.h>

//////////To move the formation
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

using std::string;
using std::vector;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// To visualize some informations
// #define COSTMAP_ON
// #define PUB_POT_MAPS
// #define FORMATION_STATE_DEBUG
////////////////

/* To use a adaptation for robotics of the method proposed by R. Silveira, E. Prestes, and L. P. Nedel,
   “Managing coherent groups,” Computer Animation and Virtual Worlds, vol. 19, no. 3-4, pp. 295–305, 2008.*/
// #define PURE_METHOD

#define NUM_ROBOTS 3//4//2

#define INITIAL_MAP 1

#ifndef PURE_METHOD
#define MAX_VEL 0.3
#else
#define MAX_VEL 0.05 // constant velocity
#endif

#define MIN_DIST 0.2 // minimum distance between goal and formation center to finish the operation

#define INC_DIST 0.2 // turtlebot size

// For callbacks
geometry_msgs::Pose robots_position[NUM_ROBOTS+1];
nav_msgs::OccupancyGrid global_costmap_recevied, map_recevied;

// For size
float map_resolution_, global_map_resolution_;
int global_map_width_, global_map_height_, global_map_origin_x_, global_map_origin_y_, global_map_origin_z_;    
int map_width_, map_height_, map_origin_x_, map_origin_y_, map_origin_z_;

// For time
ros::Time time_ant;
ros::Time initial_time;

// For pub
#ifdef PUB_POT_MAPS
nav_msgs::OccupancyGrid formation_map[NUM_ROBOTS+1];
#endif

///// Source: https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code
float constrainAngle(float x){
    float new_x = fmod(x + M_PI, 2.0 * M_PI);
    if (new_x < 0.0)
        new_x += 2.0 * M_PI;
    return new_x - M_PI;
}

class Control{
  public:
    std_msgs::Bool finished_msg;
    geometry_msgs::Pose intermediate_goal;

    void InitializePotentialField();
    void AddNeighborAsObstacle();
    void UpdatePotentialField(int robot_num);
    void SetFormationMapLimits();
    void CreateGlobalMaps();
    void ExpandObstacles();
    void FirstMoveFormation(navfn::NavfnROS *planner);
    void MoveFormation(navfn::NavfnROS *planner);
    std::tuple<float, float> GradientDescentOrientationMag(int robot_num);
    void CalculateVparamCurrent();
    bool FormationNearGoal();
    void SetInitialStartPosition(float x, float y, float z, float ang);
    void SetGoalPosition(float x, float y, float z, float ang);

    void CreateFormationMapMsg(int robot_num);
    geometry_msgs::PoseStamped CalculateRobotIdealPoseOnFormation(int robot_num);

    Control(float formation_side, float e[NUM_ROBOTS], float v[NUM_ROBOTS][2]){

      formation_side_size = formation_side;

      for(int it=0; it < NUM_ROBOTS; it++){
        pot_parameter_e[it] = e[it];
        pot_parameter_v[it][0] = v[it][0];
        pot_parameter_v[it][1] = v[it][1];

        initial_pot_parameter_v_dist_ang[it][0] = sqrt(pow(pot_parameter_v[it][0], 2.0) + pow(pot_parameter_v[it][1], 2.0));
        initial_pot_parameter_v_dist_ang[it][1] = atan2(pot_parameter_v[it][1], pot_parameter_v[it][0]);
      }

    }

    geometry_msgs::Pose formation_center_initial, formation_center;

  private:
    float dist_intermediate_center;

    float formation_ang_initial, formation_ang, formation_ang_final;

    geometry_msgs::PoseStamped start, goal;
    std::vector<geometry_msgs::PoseStamped> plan;

    vector<Cell> global_map_[NUM_ROBOTS+1];
    vector<OldCell> previously_occupied[NUM_ROBOTS+1];

    #ifdef COSTMAP_ON
    int costmap_width_, costmap_height_, costmap_origin_x_, costmap_origin_y_, costmap_origin_z_;
    #endif

    int smallest_global_i_, smallest_global_j_, largest_global_i_, largest_global_j_;

    geometry_msgs::PoseStamped robot_on_formation[NUM_ROBOTS+1];

    float pot_parameter_v_current [NUM_ROBOTS][2];

    float initial_pot_parameter_v_dist_ang [NUM_ROBOTS][2];

    float formation_side_size;

    float pot_parameter_e [NUM_ROBOTS];
    float pot_parameter_v [NUM_ROBOTS][2];

};

///////////////////////////////////
/////                         /////
/////     BASIC FUNCTIONS     /////
/////                         /////
///////////////////////////////////

// It sets width, height, origin using map_received.info
void SetupMapsDimensions(){
  // SetupVariables();
  map_width_ = map_recevied.info.width;
  map_height_ = map_recevied.info.height;
  map_origin_x_ = map_recevied.info.origin.position.x;
  map_origin_y_ = map_recevied.info.origin.position.y;
  map_origin_z_ = map_recevied.info.origin.position.z;

  global_map_width_ = map_width_;
  global_map_height_ = map_height_;
  global_map_origin_x_ = map_origin_x_;
  global_map_origin_y_ = map_origin_y_;
  global_map_origin_z_ = map_origin_z_;

  #ifdef COSTMAP_ON
  costmap_width_ = global_costmap_recevied.info.width;
  costmap_height_ = global_costmap_recevied.info.height;
  costmap_origin_x_ = global_costmap_recevied.info.origin.position.x;
  costmap_origin_y_ = global_costmap_recevied.info.origin.position.y;
  costmap_origin_z_ = global_costmap_recevied.info.origin.position.z;
  #endif
      
  map_resolution_ = map_recevied.info.resolution;
  global_map_resolution_ = map_resolution_;
  
}

// It transforms the coordinate system from the Odom to the Map
std::tuple<int, int> transformCoordinateOdomToMap(float x, float y) {
  int j = y / global_map_resolution_ - global_map_origin_y_ / global_map_resolution_;
  int i = x / global_map_resolution_ - global_map_origin_x_ / global_map_resolution_;
  return std::make_tuple(i, j);
}

// It transforms the coordinate system from the Map to the Odom
std::tuple<float, float> transformCoordinateMapToOdom(int i, int j) {
  float x = (i + global_map_origin_x_ / global_map_resolution_) * global_map_resolution_;
  float y = (j + global_map_origin_y_ / global_map_resolution_) * global_map_resolution_;
  return std::make_tuple(x, y);
}

// It converts the matrix indexes to the vector one
int matrixIndicesToVectorIndex(int i, int j) { return i + j * global_map_width_; }

// It converts the vector index to the matrix ones
std::tuple<int, int> vectorIndexToMatrixIndices(int index) {
  int temp_j = index / global_map_width_;
  int temp_i = index - temp_j * global_map_width_;
  return std::make_tuple(temp_i, temp_j);
}

///////////////////////////////////
/////                         /////
///// COMUNICATION FUNCTIONS  /////
/////                         /////
///////////////////////////////////

void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg, int n_robot){
	robots_position[n_robot] = msg->pose.pose;
}

void map_callback(const nav_msgs::OccupancyGrid& msg){
	map_recevied = msg;
}

#ifdef COSTMAP_ON
void global_costmap_callback(const nav_msgs::OccupancyGrid& msg){
    global_costmap_recevied = msg;
}
#endif


  ///////////////////////////////////
  /////                         /////
  /////      BVP FUNCTIONS      /////
  /////                         /////
  ///////////////////////////////////

  //It initializes the potencial field for each robot with the intermediate goal
  void Control::InitializePotentialField(){

    // To erase the old potential field
    int erase_border = 1.0 / global_map_resolution_;

    for(int robot_num = INITIAL_MAP; robot_num <= NUM_ROBOTS; robot_num++)
      for (int j = smallest_global_j_ - erase_border; j <= largest_global_j_ + erase_border; ++j) {
        for (int i = smallest_global_i_ - erase_border; i <= largest_global_i_ + erase_border; ++i) {
          if (i >= 0 && i < global_map_width_ && j >= 0 && j < global_map_height_) {
            int map_index = i + j * global_map_width_;
            // if it is inside the square
            if((j >= smallest_global_j_ && j <= largest_global_j_) && (i >= smallest_global_i_ && i <= largest_global_i_)){
              if(global_map_[robot_num][map_index].CELL_TYPE == kObstacle || global_map_[robot_num][map_index].CELL_TYPE == kExtraExpansion || global_map_[robot_num][map_index].CELL_TYPE == kUnknown){
                global_map_[robot_num][map_index].POTENTIAL = 1.0;
                global_map_[robot_num][map_index].BOUNDARY_CONDITION = true;
              // }else if(global_map_[robot_num][map_index].CELL_TYPE == kUnknown){
                // global_map_[robot_num][map_index].POTENTIAL = 0;
                // global_map_[robot_num][map_index].BOUNDARY_CONDITION = true;
              }else{
                global_map_[robot_num][map_index].BOUNDARY_CONDITION = false;
              }
            }else{ /// edge
              global_map_[robot_num][map_index].POTENTIAL = 1.0;
              global_map_[robot_num][map_index].BOUNDARY_CONDITION = true;
            }
          }
        }
      }

    int goal_size = 2;

    for(int r=INITIAL_MAP; r<=NUM_ROBOTS; r++){
      int i_goal, j_goal;
      std::tie(i_goal, j_goal) = transformCoordinateOdomToMap(intermediate_goal.position.x, intermediate_goal.position.y);

      for (int j = j_goal - goal_size; j <= j_goal + goal_size; ++j) {
        for (int i = i_goal - goal_size; i <= i_goal + goal_size; ++i) {          
          int map_index = i + j * global_map_width_;
          if(INITIAL_MAP == 0){
            global_map_[0][map_index].POTENTIAL = 0;
            global_map_[0][map_index].BOUNDARY_CONDITION = true;
          }
          global_map_[r][map_index].POTENTIAL = 0;
          global_map_[r][map_index].BOUNDARY_CONDITION = true;
        }
      }
    }
  }

  // It adds the robots as obstacles on the potencial fields
  void Control::AddNeighborAsObstacle(){
    float distance = 0.10;
    int map_size = global_map_[0].size(), i_neighbor_pose, j_neighbor_pose;
    int cell_distance = distance / global_map_resolution_ ;
    OldCell old_cell, old_robot, old_near_robot;

    // Restore old values
    for(int map=INITIAL_MAP; map<=NUM_ROBOTS; map++){
      while (!previously_occupied[map].empty()){
        old_cell = previously_occupied[map].back();
        global_map_[map][old_cell.map_index] = old_cell.cell;

        previously_occupied[map].pop_back();
      }
    }

    // Add robots as obstacles
    for(int r=INITIAL_MAP; r<=NUM_ROBOTS; r++){
      int i, j;
      std::tie(i, j) = transformCoordinateOdomToMap(robots_position[r].position.x, robots_position[r].position.y);
      int map_index_neighbor_pose = matrixIndicesToVectorIndex(i, j);

      for(int out_map=INITIAL_MAP; out_map<=NUM_ROBOTS; out_map++){
        if(out_map != r){
          old_robot.map_index = map_index_neighbor_pose;
          old_robot.cell = global_map_[out_map][map_index_neighbor_pose];
          previously_occupied[out_map].push_back(old_robot);

          global_map_[out_map][map_index_neighbor_pose].POTENTIAL = 1.0;
          global_map_[out_map][map_index_neighbor_pose].BOUNDARY_CONDITION = true;
          global_map_[out_map][map_index_neighbor_pose].CELL_TYPE = kNeighbor;

          std::tie(i_neighbor_pose, j_neighbor_pose)=vectorIndexToMatrixIndices(map_index_neighbor_pose);
          int i = (i_neighbor_pose - cell_distance);
          if(i<0) i=0;
          for(; i < global_map_width_ && i < i_neighbor_pose + cell_distance; i++){
            int j = j_neighbor_pose - cell_distance;
            if(j<0) j=0;
            for(; j < global_map_height_ && j < j_neighbor_pose + cell_distance; j++){
              int map_index = matrixIndicesToVectorIndex(i,j);
              if(global_map_[out_map][map_index].CELL_TYPE != kObstacle && global_map_[out_map][map_index].CELL_TYPE != kExtraExpansion &&//<= 0 &&
                sqrt(pow(i-i_neighbor_pose, 2) + pow(j-j_neighbor_pose, 2)) < cell_distance){

                old_near_robot.map_index = map_index;
                old_near_robot.cell = global_map_[out_map][map_index];
                previously_occupied[out_map].push_back(old_near_robot);

                global_map_[out_map][map_index].POTENTIAL = 1.0;
                global_map_[out_map][map_index].BOUNDARY_CONDITION = true;
                global_map_[out_map][map_index].CELL_TYPE = kNeighborExpansion;
              }
            }
          }
        }
      }
    }   
  }

  // It calculates the potencial field of the robot_num robot using parameters e and v
  void Control::UpdatePotentialField(int robot_num){

    double left, right, up, down;
    float local_v[2];

    local_v[0] = pot_parameter_v_current[robot_num-1][0] - (intermediate_goal.position.x - formation_center.position.x);
    local_v[1] = pot_parameter_v_current[robot_num-1][1] - (intermediate_goal.position.y - formation_center.position.y);

    for (int j = smallest_global_j_; j <= largest_global_j_; ++j) {
      for (int i = smallest_global_i_; i <= largest_global_i_; ++i) {
        int map_index = i + j * global_map_width_;

        if(global_map_[robot_num][map_index].BOUNDARY_CONDITION == false){
          left  = global_map_[robot_num][map_index-1].POTENTIAL;
          right = global_map_[robot_num][map_index+1].POTENTIAL;
          up    = global_map_[robot_num][map_index+global_map_width_].POTENTIAL;
          down  = global_map_[robot_num][map_index-global_map_width_].POTENTIAL;

          global_map_[robot_num][map_index].POTENTIAL = (left + right + up + down)/4.0
                                                      + (pot_parameter_e[robot_num-1]/8.0)*
                                                                (local_v[0]*(right - left)
                                                                 + local_v[1]*(up - down));
        }
      }
    }

  }


///////////////////////////////////
/////                         /////
/////    CENTRAL FUNCTIONS    /////
/////                         /////
///////////////////////////////////

// It calculates the gradient descent orientation in rads of the cell(i,j) on the robot_num global_map
std::tuple<float, float> Control::GradientDescentOrientationMag(int robot_num){
  int cell_i, cell_j;
  std::tie(cell_i, cell_j) = transformCoordinateOdomToMap(robots_position[robot_num].position.x, robots_position[robot_num].position.y);
      
  float p_x, p_y;

  int index_right, index_left, index_up, index_down, map_index;
  int global_map_size = global_map_width_ * global_map_height_;

  map_index = cell_i + cell_j * global_map_width_;
  index_right = map_index + 1;
  index_left = map_index - 1;
  index_up = map_index + global_map_width_;
  index_down = map_index - global_map_width_;

  if(index_left >= 0  &&  index_right < global_map_size){
    p_x = (-global_map_[robot_num][index_right].POTENTIAL + global_map_[robot_num][index_left].POTENTIAL)/2.0;
  }else{
    p_x = 0;
  }

  if(index_down >= 0 && index_up < global_map_size){
    p_y = (-global_map_[robot_num][index_up].POTENTIAL + global_map_[robot_num][index_down].POTENTIAL)/2.0;
  }else{
    p_y = 0;
  }

  return std::make_tuple(atan2(p_y, p_x), sqrt(p_x*p_x + p_y*p_y));
}

// It sets the limits using formation_center and formation_side_size
void Control::SetFormationMapLimits(){
  int i_formation_small, j_formation_small, i_formation_large, j_formation_large;

  std::tie(i_formation_small, j_formation_small) = transformCoordinateOdomToMap(formation_center.position.x-formation_side_size/2.0, formation_center.position.y-formation_side_size/2.0);
  std::tie(i_formation_large, j_formation_large) = transformCoordinateOdomToMap(formation_center.position.x+formation_side_size/2.0, formation_center.position.y+formation_side_size/2.0);

  if(i_formation_small>0) smallest_global_i_ = i_formation_small;
  else                    smallest_global_i_ = 0;

  if(j_formation_small>0) smallest_global_j_ = j_formation_small;
  else                    smallest_global_j_ = 0;

  if(i_formation_large<=global_map_width_-1)  largest_global_i_ = i_formation_large;
  else                                        largest_global_i_ = global_map_width_-1;

  if(j_formation_large<=global_map_height_-1) largest_global_j_ = j_formation_large;
  else                                        largest_global_j_ = global_map_height_-1;
  
}

// It creates the global_maps using map_recevied
void Control::CreateGlobalMaps(){

    #ifdef COSTMAP_ON
    if (global_map_width_ > costmap_width_ || global_map_height_ > costmap_height_) {
      int deltaX = (costmap_origin_x_ - global_map_origin_x_) / global_map_resolution_;
      int deltaY = (costmap_origin_y_ - global_map_origin_y_) / global_map_resolution_;

      deltaX = std::max(deltaX, 0);
      deltaX = std::min(deltaX, global_map_width_ - costmap_width_);
      deltaY = std::max(deltaY, 0);
      deltaY = std::min(deltaY, global_map_height_ - costmap_height_);
      // LogInfo("deltaX:{}, deltaY:{}", deltaX, deltaY);
    }
    #endif

    for(int robot_num = INITIAL_MAP; robot_num <= NUM_ROBOTS; robot_num++){
      vector<Cell> new_map;
      new_map.resize(map_width_ * map_height_);
      global_map_[robot_num].resize(global_map_width_ * global_map_height_);
      global_map_[robot_num] = new_map;

      for (int m = 0; m < global_map_height_; m++) {
        int multi = m * global_map_width_;
        for (int n = 0; n < global_map_width_; n++) {
          int i = n + multi;
          global_map_[robot_num][i].ORIGINAL_MAP = map_recevied.data[i];
 
          if (map_recevied.data[i] >= 90) {
            global_map_[robot_num][i].ALL_OBST = 100;
            global_map_[robot_num][i].SLAM_OCC = 100;
            global_map_[robot_num][i].CELL_TYPE = kObstacle;
          } else if (map_recevied.data[i] == -1) {
            global_map_[robot_num][i].ALL_OBST = -1;
            global_map_[robot_num][i].SLAM_OCC = -1;
            global_map_[robot_num][i].CELL_TYPE = kUnknown;
          } else {
            global_map_[robot_num][i].ALL_OBST = 0;
            global_map_[robot_num][i].SLAM_OCC = 0;
            global_map_[robot_num][i].CELL_TYPE = kFree;
          }

          global_map_[robot_num][i].LAST_TIME_ANALYSED = 0;
        }
      }
    }
}

// It expands the kObstacle on the global_maps
void Control::ExpandObstacles(){
  float distance = 0.25;// robot raudius = 0.105 m
  int map_size = global_map_[0].size(), i_cell, j_cell;
  int cell_distance = distance / global_map_resolution_ ;

  for(int robot_num = INITIAL_MAP; robot_num <= NUM_ROBOTS; robot_num++)
    for (int j_cell = 0; j_cell < global_map_height_; j_cell++) {
      int multi = j_cell * global_map_width_;
      for (int i_cell = 0; i_cell < global_map_width_; i_cell++) {
        int it = i_cell + multi;
        if(global_map_[robot_num][it].CELL_TYPE == kObstacle){
          int i = (i_cell - cell_distance);
          if(i<0) i=0;
          for(; i < global_map_width_ && i < i_cell + cell_distance; i++){
            int j = j_cell - cell_distance;
            if(j<0) j=0;
            for(; j < global_map_height_ && j < j_cell + cell_distance; j++){
              int map_index = matrixIndicesToVectorIndex(i,j);
              if(global_map_[robot_num][map_index].CELL_TYPE != kObstacle &&
                sqrt(pow(i-i_cell, 2) + pow(j-j_cell, 2)) < cell_distance){
                global_map_[robot_num][map_index].ALL_OBST = 100;
                global_map_[robot_num][map_index].CELL_TYPE = kExtraExpansion;
              }
            }
          }
        }
      }
    }
  }

// It sets the formation_center with initial values and calculates the first intermediate_goal using the new plan
void Control::FirstMoveFormation(navfn::NavfnROS *planner){

  ros::Time time_now = ros::Time::now();

  geometry_msgs::Quaternion new_quat;

  start.pose = formation_center;
  start.header.frame_id = "map";
  start.header.stamp=ros::Time::now();

  planner->makePlan(start, goal, plan);

  if(plan.size() < 1){
    ROS_WARN("plan size %ld, WITHOUT path to   x: %f   y: %f", plan.size(), goal.pose.position.x, goal.pose.position.y);
  }else{
    ROS_WARN("plan size %ld, path to   x: %f   y: %f", plan.size(), goal.pose.position.x, goal.pose.position.y);
  }

  float real_distance;
  int cell_distance = dist_intermediate_center/global_map_resolution_;

  if(cell_distance >= 0 && cell_distance < plan.size()){      
    real_distance = sqrt(pow(plan[cell_distance].pose.position.x - formation_center.position.x, 2.0) + pow(plan[cell_distance].pose.position.y - formation_center.position.y, 2.0));
    if(real_distance < 0.99*formation_side_size/2.0){
      intermediate_goal.position.x = plan[cell_distance].pose.position.x;
      intermediate_goal.position.y = plan[cell_distance].pose.position.y;
    }
  }

  formation_ang = atan2(intermediate_goal.position.y - formation_center.position.y, intermediate_goal.position.x - formation_center.position.x);
  
  formation_center.position = formation_center_initial.position;

  new_quat = computeQuaternionFromYaw(formation_ang);
  formation_center.orientation = new_quat;
  intermediate_goal.orientation = new_quat;

  time_ant = time_now;

}

// It moves the formation_center using the new plan, it can stop the formation
void Control::MoveFormation(navfn::NavfnROS *planner){

  ros::Time time_now = ros::Time::now();

  ros::Duration duration = time_now - initial_time, duration_int;
  double time = duration.toSec();

  geometry_msgs::Quaternion new_quat;

  start.pose = formation_center;
  start.header.frame_id = "map";
  start.header.stamp=ros::Time::now();

  planner->makePlan(start, goal, plan);

  bool stop = false;
  #ifndef PURE_METHOD

  for(int robot_num = INITIAL_MAP; robot_num <= NUM_ROBOTS; robot_num++){
    float dist_robot_form;
    // It stops the formation if the robot is too far from the ideal position
    dist_robot_form = sqrt( pow(robots_position[robot_num].position.x - robot_on_formation[robot_num].pose.position.x, 2.0) 
                          + pow(robots_position[robot_num].position.y - robot_on_formation[robot_num].pose.position.y, 2.0));
    if(dist_robot_form > formation_side_size/4.0){
      stop = true;
      break;
    }

    // It stops the formation if the robot is on the edge of the formation map
    float dist_robot_form_center;
    dist_robot_form_center = sqrt( pow(robots_position[robot_num].position.x - formation_center.position.x, 2.0) 
                          + pow(robots_position[robot_num].position.y - formation_center.position.y, 2.0));
    if(dist_robot_form_center > 0.8*formation_side_size/2.0){
      stop = true;
      break;
    }
  }  

  #endif

  if(!stop){
    float real_distance;
    int cell_distance = dist_intermediate_center/global_map_resolution_;

    if(cell_distance >= 0 && cell_distance < plan.size()){      
      real_distance = sqrt(pow(plan[cell_distance].pose.position.x - formation_center.position.x, 2.0) + pow(plan[cell_distance].pose.position.y - formation_center.position.y, 2.0));
      if(real_distance < 0.99*formation_side_size/2.0){
        intermediate_goal.position.x = plan[cell_distance].pose.position.x;
        intermediate_goal.position.y = plan[cell_distance].pose.position.y;
      }
    }
    formation_ang = atan2(intermediate_goal.position.y - formation_center.position.y, intermediate_goal.position.x - formation_center.position.x);
    
    int prox_pose = 5;

    duration_int = time_now - time_ant;
    double time_int = duration_int.toSec();

    float dist_prox_pose = sqrt(pow(plan[prox_pose].pose.position.x - formation_center.position.x, 2.0) + pow(plan[prox_pose].pose.position.y - formation_center.position.y, 2.0));
    float desloc_x, desloc_y;
    float max_desloc = time_int * MAX_VEL;

    if(dist_prox_pose < max_desloc){
      formation_center = plan[prox_pose].pose;
    }else{
      desloc_x = time_int * MAX_VEL * (plan[prox_pose].pose.position.x - formation_center.position.x)/dist_prox_pose;
      desloc_y = time_int * MAX_VEL * (plan[prox_pose].pose.position.y - formation_center.position.y)/dist_prox_pose;
      formation_center.position.x += desloc_x;
      formation_center.position.y += desloc_y;
    }

    new_quat = computeQuaternionFromYaw(formation_ang);
    formation_center.orientation = new_quat;
    intermediate_goal.orientation = new_quat;

    #ifdef FORMATION_STATE_DEBUG
    ROS_WARN("WALKING");
  }else{    
    ROS_WARN("STOPPED");
    #endif
  }
  time_ant = time_now;

}

// It calculates the pot_parameter_v_current using initial_parameter_v and formation_center.orientation
void Control::CalculateVparamCurrent(){
  float formation_ang = ComputeYaw(formation_center.orientation);
  float test_dist;
  int i_central,j_central, index, tam = 0.3 / global_map_resolution_;
  bool obst = false;

  for(int robot_num = INITIAL_MAP; robot_num <= NUM_ROBOTS; robot_num++){

    #ifndef PURE_METHOD
    // Elastic form
    obst = false;

    for(test_dist = INC_DIST; test_dist <= initial_pot_parameter_v_dist_ang[robot_num-1][0] && !obst; test_dist = test_dist + INC_DIST){
      std::tie(i_central,j_central) = transformCoordinateOdomToMap(formation_center.position.x + test_dist * cos(constrainAngle(initial_pot_parameter_v_dist_ang[robot_num-1][1] + formation_ang)), formation_center.position.y + test_dist * sin(constrainAngle(initial_pot_parameter_v_dist_ang[robot_num-1][1] + formation_ang)));
      // It verifies a square
      for(int i = i_central - tam; i <= i_central + tam && !obst; i++){
        for(int j = j_central - tam; j <= j_central + tam && !obst; j++){
          int map_index = i + j * global_map_width_;
          if(global_map_[robot_num][map_index].CELL_TYPE == kUnknown || global_map_[robot_num][map_index].CELL_TYPE == kObstacle || global_map_[robot_num][map_index].CELL_TYPE == kExtraExpansion){
            obst = true;
            break;
          }
        }
      }
    }

    test_dist = test_dist - INC_DIST;

    if(test_dist > initial_pot_parameter_v_dist_ang[robot_num-1][0])
      test_dist = initial_pot_parameter_v_dist_ang[robot_num-1][0];
    
    #else
      test_dist = initial_pot_parameter_v_dist_ang[robot_num-1][0];
    #endif

    pot_parameter_v_current[robot_num-1][0] = (test_dist) * cos(constrainAngle(initial_pot_parameter_v_dist_ang[robot_num-1][1] + formation_ang));
    pot_parameter_v_current[robot_num-1][1] = (test_dist) * sin(constrainAngle(initial_pot_parameter_v_dist_ang[robot_num-1][1] + formation_ang));
  }
}

// It verifies if the formation reached the goal
bool Control::FormationNearGoal(){

  float dist = sqrt(pow(goal.pose.position.x - intermediate_goal.position.x, 2.0) + pow(goal.pose.position.y - intermediate_goal.position.y, 2.0));

  if(dist < MIN_DIST ){
    finished_msg.data = true;
    return true;
  }else{
    finished_msg.data = false;
    return false;
  }
}

// It sets the start for the plan
void Control::SetInitialStartPosition(float x, float y, float z, float ang){
  formation_center_initial.position.x = x;
  formation_center_initial.position.y = y;
  formation_center_initial.position.z = z;
  formation_ang_initial = ang;

  formation_center = formation_center_initial;
  geometry_msgs::Quaternion new_quat;
  new_quat = computeQuaternionFromYaw(formation_ang_initial);
  formation_center.orientation = new_quat;

  start.pose = formation_center;
  start.header.frame_id = "map";
  start.header.stamp=ros::Time::now();
}

// It sets the goal for the plan adding the intermediate_goal distance
void Control::SetGoalPosition(float x, float y, float z, float ang){
  dist_intermediate_center = 0.9*formation_side_size/2.0;

  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = z;
  formation_ang_final = ang;

  ROS_WARN("FINAL CENTER x: %f   y: %f", goal.pose.position.x, goal.pose.position.y);

  geometry_msgs::Quaternion new_quat;
  new_quat = computeQuaternionFromYaw(formation_ang_final);
  goal.pose.orientation = new_quat;

  goal.header.frame_id = "map";
  goal.header.stamp = ros::Time::now();

  goal.pose.position.x += dist_intermediate_center * cos(formation_ang_final);
  goal.pose.position.y += dist_intermediate_center * sin(formation_ang_final);

  ROS_WARN("FINAL GOAL x: %f   y: %f  angle: %f", goal.pose.position.x, goal.pose.position.y, formation_ang_final);

}

// It translates POTENTIAL to [int 0, 100] for a grid
void Control::CreateFormationMapMsg(int robot_num){
  #ifdef PUB_POT_MAPS
  formation_map[robot_num].info=map_recevied.info;
  formation_map[robot_num].data.resize(global_map_height_ * global_map_width_);

  for (int m = 0; m < global_map_height_; m++) {
    int multi = m * global_map_width_;
    for (int n = 0; n < global_map_width_; n++) {
      int i = n + multi;
      formation_map[robot_num].data[i] = global_map_[robot_num][i].POTENTIAL*100;
    }
  }

  // ROS_INFO("PUBLICANDO FORMATION %d", robot_num);
  formation_map[robot_num].header.stamp=ros::Time::now();
  formation_map[robot_num].header.frame_id="map";
  #endif
}

// It calculates the ideal pose adding the formation center to v_param and returns it
geometry_msgs::PoseStamped Control::CalculateRobotIdealPoseOnFormation(int robot_num){
  robot_on_formation[robot_num].pose.orientation = formation_center.orientation;
  robot_on_formation[robot_num].pose.position.x = pot_parameter_v_current[robot_num-1][0] + formation_center.position.x;
  robot_on_formation[robot_num].pose.position.y = pot_parameter_v_current[robot_num-1][1] + formation_center.position.y;
  robot_on_formation[robot_num].pose.position.z = formation_center.position.z;
        
  return (robot_on_formation[robot_num]);
}

///////////////////////////////////
/////                         /////
/////      MAIN FUNCTION      /////
/////                         /////
///////////////////////////////////

// Principal execution
int main(int argc, char** argv){
  // Initialize
	ros::init(argc, argv, "central_control");
  ros::NodeHandle n("~");

  // Local variables for mesages
  geometry_msgs::PoseStamped pose_msg;
  std_msgs::Float64 grad_msg, grad_mag_msg;
  std_msgs::Bool finished_msg, init_operation_msg;
  geometry_msgs::PoseStamped test_point_msg, formation_center_msg;

  // Formation planner
  navfn::NavfnROS navfn;

  tf2_ros::Buffer tfBuffer(ros::Duration(10));
  tf2_ros::TransformListener tfListener(tfBuffer);
  costmap_2d::Costmap2DROS costmap("global_costmap", tfBuffer);


  // SET SUBSCRIBERS
  ros::Subscriber robots_odom_sub[NUM_ROBOTS+1], map_sub;
  
	map_sub = n.subscribe("/map", 1, map_callback);

  #ifdef COSTMAP_ON
	ros::Subscriber costmap_sub = n.subscribe("move_base/global_costmap/costmap", 1, global_costmap_callback);
	#endif

  for(int robot_num = 1; robot_num <= NUM_ROBOTS; robot_num++){
    char odom_topic[1+6+1+4+1] = {};
    strcat(odom_topic, "/robot");
    strcat(odom_topic, std::to_string(robot_num).c_str());
    strcat(odom_topic, "/odom");
    robots_odom_sub[robot_num]  = n.subscribe<nav_msgs::Odometry>(odom_topic, 1, boost::bind(&robot_odom_callback, _1, robot_num));
  }

  // SET PUBLISHERS
  ros::Publisher robot_on_formation_pub[NUM_ROBOTS+1], robot_grad_pub[NUM_ROBOTS+1], robot_grad_mag_pub[NUM_ROBOTS+1], finished_operation_pub, test_point_pub, formation_center_pub, init_operation_pub;
  #ifdef PUB_POT_MAPS
  ros::Publisher formation_map_pub[NUM_ROBOTS+1];
  #endif
  
  finished_operation_pub = n.advertise<std_msgs::Bool>("/finished_operation", 5);
  test_point_pub = n.advertise<geometry_msgs::PoseStamped>("/test_point", 5);
  formation_center_pub = n.advertise<geometry_msgs::PoseStamped>("/formation_center", 5);
  init_operation_pub = n.advertise<std_msgs::Bool>("/init_operation", 5);

  for(int robot_num = INITIAL_MAP; robot_num <= NUM_ROBOTS; robot_num++){
  #ifdef PUB_POT_MAPS
    char formation_topic[15] = {};
    strcat(formation_topic, "formation_map");
    strcat(formation_topic, std::to_string(robot_num).c_str());
   formation_map_pub[robot_num]= n.advertise<nav_msgs::OccupancyGrid>(formation_topic, 5);  
  #endif
	
    char robot_on_formation_topic[1+5+1+14] = {};
    strcat(robot_on_formation_topic, "/robot");
    strcat(robot_on_formation_topic, std::to_string(robot_num).c_str());
    strcat(robot_on_formation_topic, "_on_formation");
   robot_on_formation_pub[robot_num]= n.advertise<geometry_msgs::PoseStamped>(robot_on_formation_topic, 5);

    char robot_grad_topic[1+5+1+11] = {};
    strcat(robot_grad_topic, "/robot");
    strcat(robot_grad_topic, std::to_string(robot_num).c_str());
    strcat(robot_grad_topic, "_grad_desc");
    robot_grad_pub[robot_num]= n.advertise<std_msgs::Float64>(robot_grad_topic, 5);
    strcat(robot_grad_topic, "_mag");
    robot_grad_mag_pub[robot_num] = n.advertise<std_msgs::Float64>(robot_grad_topic, 5);
 
  }
  init_operation_msg.data = false;
  init_operation_pub.publish(init_operation_msg);
  finished_msg.data = false;
  finished_operation_pub.publish(finished_msg);

	ros::spinOnce();
  ros::Duration(5.0).sleep();
  ros::spinOnce();

  SetupMapsDimensions();

  // Start algoritm

  float e[NUM_ROBOTS] = {-0.4, -0.4, -0.4};
                        // {-0.4, -0.4, -0.4, -0.4};
                        // {-0.4, -0.4};

  float v[NUM_ROBOTS][2] = {{0.75,  1.0}, {-0.75,  1.0},  {0.0, -1.0}};
                          // {{0.75,  0.75}, {-0.75, 0.75}, {0.75,  -0.75}, {-0.75, -0.75}};
                          // {{0.5,  0.5}, {-0.5, 0.5}, {0.5,  -0.5}, {-0.5, -0.5}};
                          //{{0.0,  1.0}, {0.0, -1.0}};
                        

  float formation_side = 4.0;

  Control central_control(formation_side, e, v);

  central_control.finished_msg.data = false;
  finished_operation_pub.publish(central_control.finished_msg);

  // House
  central_control.SetInitialStartPosition(-4, 1, 0 , 0);
  // central_control.SetGoalPosition(-2.5, 1, 0, 0);
  central_control.SetGoalPosition(-4.95, 3.2, 0, 3.14);
  // central_control.SetGoalPosition(-1, 1, 0, 0);

  // // Ampulheta
  // central_control.SetInitialStartPosition(1.0, -6.0, 0.0, 1.57);
  // // central_control.SetGoalPosition(-2.0, 3.0, 0.0, 1.57); /// lower room
  // central_control.SetGoalPosition(2.0, 3.0, 0.0, 1.57); /// upper room

  // Columns
  // central_control.SetInitialStartPosition(-1.5, -1.0, 0.0, 1.57);
  // central_control.SetGoalPosition(-3.0, 5.3, 0.0, 1.57);

  initial_time = ros::Time::now();

  init_operation_msg.data = true;
  init_operation_pub.publish(init_operation_msg);

	ros::spinOnce();

  ROS_INFO("Setup OK");

  navfn.initialize("my_navfn_planner", &costmap);

  central_control.CreateGlobalMaps();

  central_control.ExpandObstacles();

  central_control.FirstMoveFormation(&navfn);

	while (ros::ok()){
	  ros::spinOnce();

    if(!central_control.FormationNearGoal()){

      central_control.MoveFormation(&navfn);

      central_control.SetFormationMapLimits();

      central_control.CalculateVparamCurrent();

      central_control.InitializePotentialField();

      for(int robot_num = INITIAL_MAP; robot_num <= NUM_ROBOTS; robot_num++)
        for(int p=0;p<150;p++)
          central_control.UpdatePotentialField(robot_num);

      central_control.AddNeighborAsObstacle();

      for(int robot_num = INITIAL_MAP; robot_num <= NUM_ROBOTS; robot_num++){
        for(int p=0;p< 10;p++)
          central_control.UpdatePotentialField(robot_num);

        #ifdef PUB_POT_MAPS
          central_control.CreateFormationMapMsg(robot_num);
          formation_map_pub[robot_num].publish(formation_map[robot_num]);
        #endif

        std::tie(grad_msg.data, grad_mag_msg.data) = central_control.GradientDescentOrientationMag(robot_num);
        robot_grad_pub[robot_num].publish(grad_msg);
        robot_grad_mag_pub[robot_num].publish(grad_mag_msg);
      
        pose_msg = central_control.CalculateRobotIdealPoseOnFormation(robot_num);
        pose_msg.header.stamp=ros::Time::now();
        pose_msg.header.frame_id="map";
        robot_on_formation_pub[robot_num].publish(pose_msg);
      }

      test_point_msg.header.stamp = ros::Time::now();
      test_point_msg.header.frame_id = "map";
      test_point_msg.pose = central_control.intermediate_goal;
      test_point_pub.publish(test_point_msg);
      formation_center_msg.header.stamp = ros::Time::now();
      formation_center_msg.header.frame_id = "map";
      formation_center_msg.pose = central_control.formation_center;
      formation_center_pub.publish(formation_center_msg);
    }
    finished_operation_pub.publish(central_control.finished_msg);

	}

  ROS_WARN("central_control finished the operation");
  return 0;
}
