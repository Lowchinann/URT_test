#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <youbot_unload_object_table/UnloadAction.h>
//#include <youbot_unload_object_table/ServerConfig.h>

#include <youbot_base_local_move/BaseAction.h>
#include <youbot_arm_joints/JointAction.h>
#include <youbot_arm_ik/IkAction.h>
#include <youbot_arm_joints_trajectory/TrajectoryAction.h>
#include <aricc_laser_pipeline/BeamDistArray.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf_conversions/tf_eigen.h>
#include <aricc_utils/geometry_utils.h>
#include <aricc_vision_msgs/Object.h>
#include <aricc_vision_msgs/ObjectArray.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <utility>
#include <iostream>
#include <assert.h>
#include <algorithm>     // std::sort

#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_RESET   "\x1b[0m"

bool objectSort(aricc_vision_msgs::Object i, aricc_vision_msgs::Object j){
  return ( fabs(i.position.y) < fabs(j.position.y) );
}
enum TaskEnum { kArmLookTable = 10,
                kOpenGripperBeforeUnload,
                kArmReadyBeforeUnload,
                kStopBase,
                kArmUnload,
                kCloseGripperUnload,
		kArmShelf, //here
                kCheckArmIkBeforeGrasp,
                kArmReadyAfterUnload,
                kArmReadyBeforeLoadTable,
                kStaticFindObjectBeforeGrasp,
                kBaseAlignObject,
                kArmLoadTable,
                kOpenGripperLoadTable,
                kMoveFindObject,
                kStaticFindObject,
                kBaseMoveLeft,
                kFinishTask,
                kArmGoUpAfterLoadTable,
                kUpdateBestList,
                kArmStandby,
                kArmStandbyHADY,  //HADY
                kEmpty 
};

struct Table{
  double height;
  double offset_x;
  double offset_y;
  double offset_z;
  double object_detect_z;
};

class Task{

public:
  TaskEnum now;
  TaskEnum last;
  std::vector< std::pair<std::string, std::string> > items;
  //std::map< std::string, std::string > items;

  Task(): now(kArmLookTable), last(kEmpty){
    items.clear();
  }
  
  ~Task(){}

  std::string toString() const{
    switch(now){
      case kArmStandby:                return "kArmStandby";
      case kArmLookTable:              return "kLookTable";
      case kOpenGripperBeforeUnload:   return "kOpenGripperBeforeUnload";
      case kArmReadyBeforeUnload:      return "kArmReadyBeforeUnload";
      case kStopBase:                  return "kStopBase";
      case kArmUnload:                 return "kArmUnload";
      case kCloseGripperUnload:        return "kCloseGripperUnload";
      case kArmShelf:                  return "kArmShelf";  //here
      case kArmReadyAfterUnload:       return "kArmReadyAfterUnload";
      case kCheckArmIkBeforeGrasp:     return "kCheckArmIkBeforeGrasp";
      case kArmReadyBeforeLoadTable:   return "kArmReadyBeforeLoadTable";
      case kArmLoadTable:              return "kArmLoadTable";
      case kOpenGripperLoadTable:      return "kOpenGripperLoadTable";
      case kMoveFindObject:            return "kMoveFindObject";
      case kBaseMoveLeft:              return "kBaseMoveLeft";
      case kArmGoUpAfterLoadTable:     return "kArmGoUpAfterLoadTable";
      case kStaticFindObject:          return "kStaticFindObject";
      case kStaticFindObjectBeforeGrasp: return "kStaticFindObjectBeforeGrasp";
      case kBaseAlignObject:           return "kBaseAlignObject";
      case kUpdateBestList:            return "kUpdateBestList";
      case kArmStandbyHADY:            return "kArmStandbyHADY"; //HADY
      case kFinishTask:                return "kFinishTask";
      
      default:
        ROS_ERROR("BUG: Unhandled State: %u", now);
        return "BUG-UNKNOWN";
    }
  }
};

class UnloadAction{

public:
  typedef actionlib::SimpleActionClient<youbot_base_local_move::BaseAction> AcBase;
  typedef actionlib::SimpleActionClient<youbot_arm_joints::JointAction>  AcArmJoints;
  typedef actionlib::SimpleActionClient<youbot_arm_ik::IkAction>
    AcArmIk;
    typedef actionlib::SimpleActionClient<youbot_arm_joints_trajectory::TrajectoryAction> AcArmTrajectory;
  typedef std::vector<trajectory_msgs::JointTrajectoryPoint> Trajectory;

  //typedef youbot_unload_object_table::ServerConfig Config;
  enum StateEnum { ACTIVE = 0, SUCCEEDED, ABORTED };

protected:
  ros::NodeHandle nh_, pnh_;
  actionlib::SimpleActionServer
    <youbot_unload_object_table::UnloadAction> action_;
  std::string action_name_;
  //boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
  boost::shared_ptr<AcArmJoints>     client_arm_joints_;
  boost::shared_ptr<AcBase>          client_base_;
  boost::shared_ptr<AcArmTrajectory> client_arm_trajectory_;
  boost::shared_ptr<AcArmIk>         client_arm_ik_; 

  boost::mutex object_mutex_;
  boost::mutex beam_mutex_;
  boost::mutex odom_mutex_;
  std::string object_topic_;
  std::string odom_topic_;
  std::string beam_topic_;

  std::string param_name_;
  std::string table_param_namespace_1_;
  std::string table_param_namespace_2_;
  std::string container_1_param_namespace_;
  std::string container_2_param_namespace_;
  
  std::vector<std::string> tray_state_;
  Task task_;
  std::vector< std::pair<std::string, std::string> > best_items_;
  std::vector< aricc_vision_msgs::Object > best_containers_;
  size_t best_slot_;

  bool is_arm_sent_goal_;
  bool is_base_sent_goal_;
  bool is_new_goal_;

  double rate_;
  double timeout_;
  double offset_x_;
  double offset_y_;
  double offset_z_;
  double offset_roll_;
  double object_detect_z_;
  double dist_;
  double dist_left_;
  double robot_width_;

  unsigned int cnt_find_object_;
  unsigned int cnt_arm_move_;
  std::vector<aricc_vision_msgs::ObjectArray> object_msgs_;
  nav_msgs::Odometry odom_msg_;
  aricc_laser_pipeline::BeamDistArray beam_msg_;

  Trajectory armTrajectoryLookTable_;
  Trajectory armTrajectoryStandby_;
  Trajectory armTrajectoryNavigationPose_;
  Trajectory armTrajectoryArmShelf_; //here
  Trajectory armTrajectoryMidpoint_1_; //here
  Trajectory armTrajectoryMidpoint_2_; //here
  Trajectory armTrajectoryMidpoint_3_; //here
  Trajectory armTrajectoryAfterUnload_; //here
  Trajectory armTrajectoryGoTray_1_;
  Trajectory armTrajectoryGoTray_2_;
  Trajectory armTrajectoryGoTray_3_;
  Trajectory armTrajectoryLeaveTray_1_;
  Trajectory armTrajectoryLeaveTray_2_;
  Trajectory armTrajectoryLeaveTray_3_;
  Trajectory armTrajectoryReadyLeaveTray_1_;
  Trajectory armTrajectoryReadyLeaveTray_2_;
  Trajectory armTrajectoryReadyLeaveTray_3_;


  std::vector<double> gripperOpen_; 
  std::vector<double> gripperClose_; 
  
  std::vector<Table> table_list_;

  youbot_unload_object_table::UnloadGoal goal_;
  youbot_unload_object_table::UnloadFeedback feedback_;
  youbot_unload_object_table::UnloadResult result_;

  ros::Subscriber sub_odom_, sub_object_, sub_beam_; 
  ros::Time time_now_, time_start_;
  geometry_msgs::Pose2D odom_now_;
  geometry_msgs::Pose2D odom_last_;
  geometry_msgs::Pose2D odom_start_;
  geometry_msgs::Pose2D odom_dist_;
 
public:
  UnloadAction(std::string name) :
    pnh_("~"),is_arm_sent_goal_(false),is_base_sent_goal_(false),is_new_goal_(true), cnt_find_object_(0),cnt_arm_move_(0), dist_(0),offset_x_(0),offset_y_(0),offset_roll_(0), action_(nh_, name, boost::bind(&UnloadAction::executeCB, this, _1), false),
    action_name_(name){
  }

  ~UnloadAction(void){
  }

  bool loadTables(){
    table_list_.clear();
    //Loading tasks from config files
    ROS_INFO("----------");
    ROS_INFO("Loading workstations ...");
    XmlRpc::XmlRpcValue table_list;

    if( pnh_.getParam("workstations", table_list) ){
      ROS_INFO("Found table list, size:%d",table_list.size());
      ROS_ASSERT(table_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      if( table_list.getType() == XmlRpc::XmlRpcValue::TypeArray ){
        for( size_t i = 0; i < table_list.size(); ++i){
          std::string height =
          std::string(table_list[i]["table_height"]);
          std::string offset_x =
          std::string(table_list[i]["offset_x"]);
          std::string offset_y =
          std::string(table_list[i]["offset_y"]);
          std::string offset_z =
          std::string(table_list[i]["offset_z"]);
          std::string object_detect_z =
          std::string(table_list[i]["object_detect_z"]);
          ROS_INFO("%s, %s, %s, %s, %s", height.c_str(),
            offset_x.c_str(), offset_y.c_str(),offset_z.c_str(),
            object_detect_z.c_str());
          Table tb;
          tb.height  = atof(height.c_str());
          tb.offset_x   = atof(offset_x.c_str());
          tb.offset_y   = atof(offset_y.c_str());
          tb.offset_z   = atof(offset_z.c_str());
          tb.object_detect_z = atof(object_detect_z.c_str());
          table_list_.push_back(tb);
        }
        ROS_INFO("Loaded %lu tables",table_list_.size());
        return true;
      }
    }
    else ROS_ERROR("Couldn`t load tables");
    ROS_INFO("----------");
    return false;
  }

  bool initAction(){
    ROS_INFO("----------");
    //srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (pnh_);
    //dynamic_reconfigure::Server<Config>::CallbackType f =
    //  boost::bind (&UnloadAction::configCallback, this, _1, _2);
    //srv_->setCallback(f);

    pnh_.param<double>("rate", rate_, 20.0);
    pnh_.param<double>("scan_dist",   dist_, 0.06);
    pnh_.param<double>("robot_width", robot_width_, 0.50);
    
    pnh_.param<std::string>
      ("object_topic", object_topic_, "detected_objects");
    pnh_.param<std::string>
      ("odom_topic", odom_topic_, "odom");
    pnh_.param<std::string>
      ("beam_topic", beam_topic_, "beam");

    pnh_.param("table_param_namespace_1", table_param_namespace_1_, std::string("/youbot_object_detection/RotatedRectFinder/"));
    pnh_.param("table_param_namespace_2", table_param_namespace_2_, std::string("/youbot_object_detection/RotatedRectFinder/"));
    pnh_.param("container_1_param_namespace", container_1_param_namespace_, std::string("/youbot_object_detection/RotatedRectFinder/"));
    pnh_.param("container_2_param_namespace", container_2_param_namespace_, std::string("/youbot_object_detection/RotatedRectFinder/"));
    pnh_.param("param_name", param_name_, std::string("z"));

    //if(!loadAllArmJoints()) return false;
    if(!loadAllArmTrajectories()) return false;
    if(!loadTables()) return false;

    client_base_.reset(new AcBase("youbot_base_local_move_server", true));
    client_arm_joints_.reset(new AcArmJoints("youbot_arm_joints_server", true));
    client_arm_ik_.reset(new AcArmIk("youbot_arm_ik_server", true));
    client_arm_trajectory_.reset(new AcArmTrajectory("youbot_arm_joints_trajectory_server", true));

    unsigned int cnt = 0;
    while(!client_base_->waitForServer(ros::Duration(5.0))){
      ROS_ERROR("waiting for youbot_base_local_move actionlib");
      if(++cnt == 5) return false;
    }
    cnt = 0;
    while(!client_arm_joints_->waitForServer(ros::Duration(5.0))){
      ROS_ERROR("waiting for youbot_arm_joints actionlib");
      if(++cnt == 5) return false;
    }
     cnt = 0;
    while(!client_arm_ik_->waitForServer(ros::Duration(5.0))){
      ROS_ERROR("waiting for youbot_arm_ik actionlib");
      if(++cnt == 5) return false;
    }
    cnt = 0;
    while(!client_arm_trajectory_->waitForServer(ros::Duration(5.0))){
      ROS_ERROR("waiting for youbot_arm_trajectory actionlib");
      if(++cnt == 5) return false;
    }

    action_.start();
    object_msgs_.clear();
    ROS_INFO("Starting %s ...", action_name_.c_str());
    return true;
  }

  void subObject(){
    sub_object_ = nh_.subscribe( object_topic_,1, &UnloadAction::objectCB,this);
  }

  void subOdom(){
    sub_odom_   = nh_.subscribe( odom_topic_,  1, &UnloadAction::odomCB,this);
  }

  void subBeam(){
    sub_beam_   = nh_.subscribe( beam_topic_,  1, &UnloadAction::beamCB,this);
  }

  void unsubObject(){
    sub_object_.shutdown();
    object_msgs_.clear();
  }

  void unsubOdom(){
    sub_odom_.shutdown();
  }

  void unsubBeam(){
    sub_beam_.shutdown();
  }

  void unsubscribe(){
    unsubObject();
    unsubOdom();
    unsubBeam();
  }

  bool loadAllArmTrajectories(){
    if(!loadArmTrajectory("arm_trajectory_look_table",armTrajectoryLookTable_))
      return false;
    if(!loadArmTrajectory("arm_trajectory_standby",armTrajectoryStandby_))
      return false;
    if(!loadArmTrajectory("arm_trajectory_navigation_pose",armTrajectoryNavigationPose_))
      return false;
    if(!loadArmTrajectory("arm_trajectory_arm_shelf",armTrajectoryArmShelf_))  //here
      return false;  
    if(!loadArmTrajectory("arm_trajectory_midpoint_1",armTrajectoryMidpoint_1_))  //here
      return false;
    if(!loadArmTrajectory("arm_trajectory_midpoint_2",armTrajectoryMidpoint_2_))  //here
      return false;
    if(!loadArmTrajectory("arm_trajectory_midpoint_3",armTrajectoryMidpoint_3_))  //here
      return false;
    if(!loadArmTrajectory("arm_trajectory_after_unload",armTrajectoryAfterUnload_))  //here
      return false;
    if(!loadArmTrajectory("arm_trajectory_go_tray_1",armTrajectoryGoTray_1_))
      return false;
    if(!loadArmTrajectory("arm_trajectory_go_tray_2",armTrajectoryGoTray_2_))
      return false;
    if(!loadArmTrajectory("arm_trajectory_go_tray_3",armTrajectoryGoTray_3_))
      return false;
    if(!loadArmTrajectory("arm_trajectory_leave_tray_1",armTrajectoryLeaveTray_1_))
      return false;
    if(!loadArmTrajectory("arm_trajectory_leave_tray_2",armTrajectoryLeaveTray_2_))
      return false;
    if(!loadArmTrajectory("arm_trajectory_leave_tray_3",armTrajectoryLeaveTray_3_))
      return false;
    if(!loadArmTrajectory("arm_trajectory_ready_leave_tray_1",armTrajectoryReadyLeaveTray_1_))
      return false;
    if(!loadArmTrajectory("arm_trajectory_ready_leave_tray_2",armTrajectoryReadyLeaveTray_2_))
      return false;
    if(!loadArmTrajectory("arm_trajectory_ready_leave_tray_3",armTrajectoryReadyLeaveTray_3_))
      return false;
    return true;
  }
  
  bool loadArmTrajectory(std::string name, Trajectory& traj){
      try{
        XmlRpc::XmlRpcValue list;
        if( !pnh_.hasParam(name) ){
          ROS_ERROR("Cannot find %s", name.c_str());
          return false;
        }
        pnh_.getParam( name, list );
        ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_INFO("%s has %d waypoints:",name.c_str(), list.size());
        traj.clear();
        traj.resize(list.size());

        for(unsigned int i = 0; i < list.size(); ++i){
          traj[i].positions.resize(5);
          std::string d = static_cast<std::string>(list[i]["duration"]);
          traj[i].time_from_start = ros::Duration(atof(d.c_str()));
          XmlRpc::XmlRpcValue joint_list = list[i]["joints"];
          //ROS_INFO("%d joint list found", joint_list.size());
          for(unsigned int j = 0; j < joint_list.size(); ++j){
            std::string value = static_cast<std::string>(joint_list[j]);
            traj[i].positions[j] = atof(value.c_str());
          }
        }
      }
      catch(ros::Exception e){
        ROS_ERROR("%s", e.what());
        return false;
      }
      return true;
  }

  void objectCB(const aricc_vision_msgs::ObjectArray::ConstPtr& msg){
    boost::mutex::scoped_lock lock(object_mutex_);
    ros::Time now = ros::Time::now();
    double time_diff = (now - msg->header.stamp).toSec();
    if(fabs(time_diff) < 0.5){
      object_msgs_.push_back(*msg);
      //ROS_WARN("Have %lu object msgs, have %lu objects", 
      //object_msgs_.size(), msg->objects.size());
    }
    /*
    else{ 
      ROS_WARN("%s: Data is older %.3lf than now", action_name_.c_str(), time_diff);
    }*/

  }

   void odomCB(const nav_msgs::Odometry::ConstPtr& msg){
    boost::mutex::scoped_lock lock(odom_mutex_);
    odom_msg_ = *msg;
  }

  void beamCB(const aricc_laser_pipeline::BeamDistArray::ConstPtr& msg){
    boost::mutex::scoped_lock lock(beam_mutex_);
    beam_msg_ = *msg; 
  }

  //void configCallback(Config &config, uint32_t level){
  //  boost::mutex::scoped_lock lock(mutex_);
  //}

  StateEnum moveGripper( std::string cmd ){
    if(!is_arm_sent_goal_){
      youbot_arm_joints::JointGoal goal;
      std::vector<double> joints;
      joints.resize(2);
      if(cmd == "open"){
        joints[0] = 1;
        joints[1] = 1;
      }
      else if(cmd == "close_light"){
        joints[0] = 0.0;
        joints[1] = 0.0;
      }
      else if(cmd == "close_heavy"){
        joints[0] = 0.0;
        joints[1] = 0.0;
      }
      goal.timeout = 5;
      goal.name = "gripper";
      goal.joints = joints;
      sleep(1.0);
      client_arm_joints_->sendGoal(goal);
      is_arm_sent_goal_ = true;
      return ACTIVE;
    }
    if( client_arm_joints_->getState().toString() == "SUCCEEDED" ){ 
      is_arm_sent_goal_ = false;   
      return SUCCEEDED;
    }
    else if( client_arm_joints_->getState().toString() == "ABORTED" ){ 
      is_arm_sent_goal_ = false;
      return ABORTED; 
    }
    else return ACTIVE; 
  }

  StateEnum initArm(){
    youbot_arm_joints_trajectory::TrajectoryGoal goal;
    goal.timeout = 20;
    goal.points.clear();
    //HADY VERY VERY VERY IMPORTANT!!! FINALLY WORK FOR WALLS IN MONTREAL
    goal.points = armTrajectoryStandby_; //HADY VERY VERY VERY IMPORTANT!!! FINALLY WORK FOR WALLS IN MONTREAL
    //goal.points = armTrajectoryLookTable_;
    client_arm_trajectory_->sendGoal(goal);
    client_arm_trajectory_->waitForResult(ros::Duration(20));
    if( client_arm_trajectory_->getState().toString() == "SUCCEEDED" ){
      is_arm_sent_goal_ = false;
      return SUCCEEDED;
    }
    else if( client_arm_trajectory_->getState().toString() == "ABORTED" ){
      is_arm_sent_goal_ = false;
      return ABORTED;
    }
  }

  StateEnum moveArmTrajectory( Trajectory traj ){
   //ROS_INFO("Arm trajectory is running %d",is_arm_sent_goal_);
   if(!is_arm_sent_goal_){
      youbot_arm_joints_trajectory::TrajectoryGoal goal;
      goal.timeout = 10;
      goal.points = traj;
      client_arm_trajectory_->sendGoal(goal);
      is_arm_sent_goal_ = true;
      return ACTIVE;
    }
    if( client_arm_trajectory_->getState().toString() == "SUCCEEDED" ){
      is_arm_sent_goal_ = false;
      return SUCCEEDED;
    }
    else if( client_arm_trajectory_->getState().toString() == "ABORTED" ){
      is_arm_sent_goal_ = false;
      return ABORTED;
    }
    else return ACTIVE;
  }
  
  StateEnum moveArmJoints( std::vector<double> joints ){
    if(!is_arm_sent_goal_){
      youbot_arm_joints::JointGoal goal;
      goal.timeout = 20;
      goal.name = "arm";
      goal.joints = joints;
      client_arm_joints_->sendGoal(goal);
      is_arm_sent_goal_ = true;
      return ACTIVE;
    }
    if( client_arm_joints_->getState().toString() == "SUCCEEDED" ){ 
      is_arm_sent_goal_ = false;   
      return SUCCEEDED;
    }
    else if( client_arm_joints_->getState().toString() == "ABORTED" ){ 
      is_arm_sent_goal_ = false;
      return ABORTED; 
    }
    else return ACTIVE; 
  }

  StateEnum moveArmIk( geometry_msgs::Point pos, 
    geometry_msgs::Vector3 ori,
    bool move_arm = true ){
    if(!is_arm_sent_goal_){
      youbot_arm_ik::IkGoal goal;
      
      if(pos.z >= 0.2) pos.z = 0.2;
      //if(fabs(pos.x) >= 0.24) pos.x = (pos.x > 0)? pos.x = 0.24: pos.x = -0.24;

      goal.timeout = 10;
      goal.position = pos;
      goal.orientation = ori;
      goal.name = "preferred_pitch_ik";
      //goal.name = "closest_ik";
      goal.pitch = M_PI_2;
      goal.duration = 5.0;
      goal.move_arm = move_arm;
      client_arm_ik_->sendGoal(goal);
      is_arm_sent_goal_ = true;
      return ACTIVE;
    }
    if( client_arm_ik_->getState().toString() == "SUCCEEDED" ){
      is_arm_sent_goal_ = false;
      return SUCCEEDED;
    }
    else if( client_arm_ik_->getState().toString() == "ABORTED" ){ 
      is_arm_sent_goal_ = false;
      return ABORTED;
    }
    else return ACTIVE;
  }

  StateEnum moveArmUpDownIk(aricc_vision_msgs::Object object, 
    int dir, bool move_arm = true){
    StateEnum state;
    geometry_msgs::Point pos = object.position;
    geometry_msgs::Vector3 ori;
    std::string key_1 = "Container";
    std::string key_2 = "Table";
    std::size_t found_1 = object.name.rfind(key_1);
    std::size_t found_2 = object.name.rfind(key_2);
    double d = 0.0;
    double move_side = 0.05;
    //Container
    if(found_1 != std::string::npos){
      pos.x += offset_x_;
      //pos.y += offset_y_;
      pos.y = 0;
      //change the value to be added to offset_z_ to adjust dropping height
      pos.z = offset_z_ + 0.088; //0.095
      ori.x = 0.0 + offset_roll_;
      ori.y = M_PI_2;
      ori.z = 0.0;
    }
    //Table
    if(found_2 != std::string::npos){
      pos.x = offset_x_;
      //pos.y += offset_y_;
      pos.y = 0;
      pos.z = offset_z_;
      ori.x = M_PI_2 + offset_roll_;
      ori.y = M_PI_2;
      ori.z = 0.0;
    }

    //ROS_INFO("OFFSET_Z:%.3lf",offset_z_);
    //move arm up
    if(dir == 1) {
      //Container
      if(found_1 != std::string::npos){
        d = 0;
        pos.z += d;
      }
      //Table
      if(found_2 != std::string::npos){
        d = 0;
        pos.z += d;
      }
      state = moveArmIk(pos, ori, move_arm);
      return state;
    }

    //move arm down
    if(dir == -1){
      //Container
      if(found_1 != std::string::npos){
        d = 0.0;
        if(cnt_arm_move_ < 2){
          double offset = cnt_arm_move_* d;
          pos.z += d - offset;
          state = moveArmIk(pos, ori, move_arm);
        }
        if(state == SUCCEEDED) cnt_arm_move_++;
        else if(state == ABORTED) return state;

        if(cnt_arm_move_ == 1){
          cnt_arm_move_ = 0;
          return state;
        }
      }  

      //Table
      if(found_2 != std::string::npos){
        d = 0.04;
        if(cnt_arm_move_ < 2){
          double offset = cnt_arm_move_* d;
          pos.z += d - offset;
          state = moveArmIk(pos, ori, move_arm);
        }
        else{
          pos.x += move_side;
          pos.z = offset_z_;
          state = moveArmIk(pos, ori, move_arm);
        }

        if(state == SUCCEEDED) cnt_arm_move_++;
        else if(state == ABORTED) return state;
        if(cnt_arm_move_ == 2){
          cnt_arm_move_ = 0;
          return state;
        }
        return ACTIVE;
      }
    }
  }

   StateEnum stopBase(){
    geometry_msgs::Pose2D pose;
    pose.x = 0.0;
    pose.y = 0.0;
    pose.theta = 0.0;
    youbot_base_local_move::BaseGoal goal;
    goal.timeout = 60;
    goal.destination = pose;
    client_base_->sendGoal(goal);
    client_base_->waitForResult(ros::Duration(0.5));
    if( client_base_->getState().toString() == "SUCCEEDED" ){
      is_base_sent_goal_ = false;
      return SUCCEEDED;
    }
    else if( client_base_->getState().toString() == "ABORTED" ){
      is_base_sent_goal_ = false;
      return ABORTED;
    }
  }  

  StateEnum moveBase( geometry_msgs::Pose2D pose){
    if(!is_base_sent_goal_){
      youbot_base_local_move::BaseGoal goal;
      goal.timeout = 60;
      goal.destination = pose;
      client_base_->sendGoal(goal);
      is_base_sent_goal_ = true;
      return ACTIVE;
    }
    if( client_base_->getState().toString() == "SUCCEEDED" ){ 
      is_base_sent_goal_ = false;   
      return SUCCEEDED;
    }
    else if( client_base_->getState().toString() == "ABORTED" ){ 
      is_base_sent_goal_ = false;
      return ABORTED; 
    }
    else return ACTIVE; 
  }

  StateEnum findItemAndContainer( 
    std::vector< std::pair<std::string, std::string> > items,
    std::vector< std::pair<std::string, std::string> >& best_items,
    std::vector< aricc_vision_msgs::Object> & best_containers ){
    boost::mutex::scoped_lock lock(object_mutex_);

    if( ++cnt_find_object_ >= 30 ) {
      cnt_find_object_ = 0;
      object_msgs_.clear();
     return ABORTED;
    }
    
    ros::Time now = ros::Time::now();
    std::vector< aricc_vision_msgs::Object > detected_objects;
    detected_objects.clear();
  /*  
    if(object_msgs_.size() == 0){
      ROS_WARN("No object msgs received");
      return ACTIVE;
    }*/

    //ros::Time now = ros::Time::now();
    //std::vector< aricc_vision_msgs::Object > detected_objects;
    //detected_objects.clear();
    for( std::vector< aricc_vision_msgs::ObjectArray >::reverse_iterator rit = object_msgs_.rbegin(); rit != object_msgs_.rend(); ++rit){
      double time_diff = (now - rit->header.stamp).toSec();
      if(time_diff > 0.5){
        //it = object_msgs_.erase(it);
        //ROS_WARN("%s: Data is older than %.3lf", action_name_.c_str(), time_diff);
        continue;
      }

      for(std::vector< aricc_vision_msgs::Object >::iterator itt = rit->objects.begin(); itt != rit->objects.end(); ++itt){
         if(fabs(itt->position.y) >= 0.10) continue;
         detected_objects.push_back(*itt);
      }
    }

    int objects_size = detected_objects.size();
    if( objects_size == 0 ){
      //ROS_WARN("%s: Cannot receive object. size: %d/%lu", action_name_.c_str(), objects_size,object_msgs_.size());
      return ACTIVE;
    }
     
    std::string str = "detected objects:[";
    for(size_t i = 0; i < detected_objects.size(); ++i){
      str += detected_objects.at(i).name;
      str += " ";
    }
    str += "]";
    ROS_INFO("%s",str.c_str());
    
    std::vector<aricc_vision_msgs::Object> cd_objects;
    cd_objects.clear();

    for(std::vector< aricc_vision_msgs::Object >::iterator it = detected_objects.begin(); it != detected_objects.end(); ++it){
      for( std::vector< std::pair<std::string, std::string> >::iterator itt = items.begin(); itt != items.end(); ++itt) {
        if( it->name == itt->second ) {
          cd_objects.push_back(*it);
          break;
        }
      }
    }
    if(cd_objects.size() == 0) return ACTIVE;
    
    str = "candidate objects:\n[";
    for(size_t i = 0; i < cd_objects.size(); ++i){
      str += cd_objects.at(i).name;
      str += " ";
    }
    str += "]";
    ROS_INFO("%s",str.c_str());
    
    //Sort objects by absolute distance
    std::sort(cd_objects.begin(), cd_objects.end(),objectSort);
    //ROS_INFO("cd_objects:%lu, items:%lu",cd_objects.size(), items.size());
    std::vector<aricc_vision_msgs::Object> temp_containers;
    temp_containers.clear();
    temp_containers.push_back(cd_objects[0]);
    for(std::vector< aricc_vision_msgs::Object >::iterator it = cd_objects.begin(); it != cd_objects.end(); ++it){
      bool same_name = false;
      for(std::vector< aricc_vision_msgs::Object >::iterator itt = temp_containers.begin(); itt != temp_containers.end(); ++itt){
        if(it->name == itt->name) same_name = true;
      }
      if(!same_name) temp_containers.push_back(*it);
    }

    best_containers.clear();
    best_items.clear();
    for ( std::vector< std::pair<std::string,std::string> >::iterator it=items.begin(); it!=items.end(); ++it){
      for(std::vector< aricc_vision_msgs::Object >::iterator itt = temp_containers.begin(); itt != temp_containers.end(); ++itt){
        if(it->second == itt->name){
          best_containers.push_back(*itt);
          best_items.push_back(*it);
        }
      }
    }
    for(std::vector< aricc_vision_msgs::Object >::iterator it = best_containers.begin(); it != best_containers.end(); ++it){
      ROS_INFO("best_container: %s:[%.3lf,%.3lf,%.3lf]",
        it->name.c_str(),
        it->position.x, it->position.y,
        aricc_utils::rad2Deg(it->orientation.z));
    }
    for(std::vector< std::pair<std::string,std::string> >::iterator it=best_items.begin(); it!=best_items.end(); ++it){
      ROS_INFO("best_item: [%s, %s]",
        it->first.c_str(),
        it->second.c_str());
    }
    cnt_find_object_ = 0;
    object_msgs_.clear();
    return SUCCEEDED;
  }

  void updateItemListAndTray(
    std::vector<std::pair<std::string,std::string> >& items, 
    std::vector<std::string>& tray, 
    std::pair<std::string, std::string> best_item,
    size_t slot){
    tray[slot] = "empty";
    std::vector< std::pair<std::string,std::string> >::iterator it;
    for( std::vector< std::pair<std::string,std::string> >::iterator it=items.begin(); it!=items.end(); ){
      if(it->first == best_item.first &&
        it->second == best_item.second ) {
          items.erase(it);
          break;
        } 
      else ++it;
    }
    printItemsAndTray(items, tray);
  }

  size_t getTraySlot( std::pair<std::string, std::string> item, 
    std::vector<std::string> tray ){
    for( size_t i = 0; i < tray.size(); ++i ){
      if(tray.at(i) == item.first) return i;
    }
  }

  bool isBaseMoveEnough(double dist = 0){
    boost::mutex::scoped_lock lock(beam_mutex_);
    double left_dist = 0;
    double left_angle = 89; 

    double time_diff = 
      (ros::Time::now() - beam_msg_.header.stamp).toSec();
    if( time_diff > 0.5 ){ 
      ROS_WARN("Beam msg: Data is older than %.3lf",  time_diff); 
      //return false;
      left_dist = DBL_MAX;
    }
    else{
      for(unsigned int i = 0; i < beam_msg_.values.size(); ++i){
        if(beam_msg_.values[i].angle == left_angle){ 
          left_dist = beam_msg_.values[i].distance;
        }
      }
    }
    
    double limit_dist = left_dist - (robot_width_/2.0) ;
    double dist_1 = limit_dist - dist;
    double dist_2 = dist_left_ -dist;
    double dist_tol = 0.01;
    
    //ROS_INFO("laser distance:%.3lf, robot left distance:%.3lf", dist_1, dist_2);
    
    if( dist_1 <= dist_tol || dist_2 <= dist_tol){
      ROS_WARN("laser distance:%.3lf, robot left distance:%.3lf", dist_1, dist_2);
      return true; 
    }
    return false;
  }

  StateEnum taskStep(){
    if( isBaseMoveEnough() ){
      ROS_WARN("Robot has moved %.3lf, task succeeded", dist_);
      return SUCCEEDED;
    }

    //return ACTIVE;

    if( task_.now != task_.last )
      ROS_INFO( ANSI_COLOR_GREEN "\n< -------- Task now: %s -------->\n" ANSI_COLOR_RESET,task_.toString().c_str() );
    task_.last = task_.now;

    StateEnum state;
    geometry_msgs::Pose2D basePose;
    switch(task_.now){

      case kArmStandbyHADY: //HADY 
	state = moveArmTrajectory(armTrajectoryStandby_);
	if( state == SUCCEEDED ) task_.now = kArmLookTable;
	else if(state == ABORTED) return ABORTED;
      break;

      case kArmLookTable:   
        state = moveArmTrajectory(armTrajectoryLookTable_);
        if(state == SUCCEEDED){ //ASAD
          task_.now = kStaticFindObject;
          sleep(2.0);
       }
      break;

      case kStaticFindObject:
        state = findItemAndContainer( task_.items,
          best_items_, best_containers_ );
       if(state == SUCCEEDED) task_.now = kCheckArmIkBeforeGrasp;
        else if(state == ABORTED) task_.now = kBaseMoveLeft;
      break;
      

      case kBaseMoveLeft:
        basePose.x = 0.0; 
        basePose.y = 0.1; 
        basePose.theta = 0.0;
        //ROS_INFO("%.3lf",dist_left_);
        state = moveBase(basePose);
        if(state == SUCCEEDED) task_.now = kStopBase;
        //else if(state == ABORTED) return ABORTED;
      break;

      case kStopBase:
        state = stopBase();
        if( state == SUCCEEDED ) task_.now = kArmLookTable;
        else if(state == ABORTED) return ABORTED;
      break;


      case kCheckArmIkBeforeGrasp:
        state = moveArmUpDownIk(best_containers_[0], -1, false);
        if(state == SUCCEEDED){ 
          task_.now = kArmUnload;
        }
        else if(state == ABORTED){ 
          task_.now = kUpdateBestList;
        }
      break;

      case kArmUnload:
	best_slot_ = getTraySlot( best_items_[0], tray_state_);
        if(best_slot_ == 0) 
          state = moveArmTrajectory(armTrajectoryLeaveTray_1_);
        if(best_slot_ == 1) 
          state = moveArmTrajectory(armTrajectoryLeaveTray_2_);
        if(best_slot_ == 2) 
          state = moveArmTrajectory(armTrajectoryLeaveTray_3_);
        if(state == SUCCEEDED){ 
          task_.now = kCloseGripperUnload;
        }
      break;

      case kCloseGripperUnload:
        if(best_items_[0].first == "S40_40_G" ||
           best_items_[0].first == "S40_40_B" ||
           best_items_[0].first == "M30")
          state = moveGripper("close_light");
        else state = moveGripper("close_heavy");
        if(state == SUCCEEDED) {
          task_.now = kArmReadyAfterUnload;
	//if(state == SUCCEEDED) task_.now = kArmShelf;
        }
      break;

     
      case kArmReadyAfterUnload:
        if(best_slot_ == 0) 
          state = moveArmTrajectory(armTrajectoryReadyLeaveTray_1_);
        if(best_slot_ == 1) 
          state = moveArmTrajectory(armTrajectoryReadyLeaveTray_2_);
        if(best_slot_ == 2) 
          state = moveArmTrajectory(armTrajectoryReadyLeaveTray_3_);
        if(state == SUCCEEDED) task_.now = kArmLoadTable;
       //if(state == SUCCEEDED) task_.now = kArmShelf;
        //else if(state == ABORTED) return ABORTED;
      break;

      case kArmReadyBeforeLoadTable:
     state = moveArmTrajectory(armTrajectoryLookTable_);
        if(state == SUCCEEDED) task_.now = kArmLoadTable;
       //if(state == SUCCEEDED) task_.now = kArmShelf;
        //else if(state == ABORTED) return ABORTED;
      break;


  /*  ////here arm shelf code//// */
       case kArmShelf:
        state = moveArmTrajectory(armTrajectoryArmShelf_);
        if(state == SUCCEEDED) {
         task_.now = kArmLoadTable;
       } 
      break;

      case kArmLoadTable:
      //state = moveArmTrajectory(armTrajectoryArmShelf_);
        state = moveArmUpDownIk(best_containers_[0], -1, true);
        if(state == SUCCEEDED) task_.now = kOpenGripperLoadTable;
        //else if(state == ABORTED) task_.now = kArmLookTable;//ASAD
      break;

      case kOpenGripperLoadTable:
        state = moveGripper("open");
        if(state == SUCCEEDED) task_.now = kArmStandby;
        //else if(state == ABORTED) return ABORTED;
      break;
  
      case kArmGoUpAfterLoadTable:
       //state = moveArmTrajectory(armTrajectoryAfterUnload_);//??
       state = moveArmUpDownIk(best_containers_[0], 1);
        if(state == SUCCEEDED) task_.now = kArmStandby;
        else if(state == ABORTED) task_.now = kArmStandby;
      break;

      case kArmStandby:
        //state = moveArmTrajectory(armTrajectoryAfterUnload_);//SHELF?
        state = moveArmTrajectory(armTrajectoryLookTable_);
        if(state == SUCCEEDED) {
          updateItemListAndTray( task_.items, tray_state_,
            best_items_[0], best_slot_);
          if(task_.items.size() == 0) task_.now = kFinishTask;
          else task_.now = kUpdateBestList;
        }
      break;
      
      case kUpdateBestList:
        if(best_containers_.size() != 0 && 
          best_items_.size() != 0){
          best_containers_.erase (best_containers_.begin());
          best_items_.erase (best_items_.begin());
        }
        ROS_INFO("Have %lu best containers",best_containers_.size());
        if(best_containers_.size() == 0 &&
           best_items_.size() == 0)
          task_.now = kFinishTask;//kBaseMoveLeft;
        else
          task_.now = kBaseMoveLeft;//kCheckArmIkBeforeGrasp;

      break;

      case kFinishTask:
	  ROS_INFO("BEFORE STATE");
          state = moveArmTrajectory(armTrajectoryStandby_);
	  ROS_INFO("AFTER STATE");
          //state = moveArmTrajectory(armTrajectoryAfterUnload_);
          //state = moveArmTrajectory(armTrajectoryNavigationPose_);
        //return SUCCEEDED;
        if(state == SUCCEEDED) return SUCCEEDED;
        else if(state == ABORTED) return ABORTED;
    }
    return ACTIVE;
  }

  void resetState(){
    //ROS_INFO("resetState");
    stopBase();
    initArm();
    is_arm_sent_goal_ = false;
    is_base_sent_goal_ = false;
    is_new_goal_ = true;

    time_start_ = ros::Time::now();
    //task_.now = kArmStandbyHADY; //HADY
    task_.now = kArmLookTable;
    client_arm_joints_->cancelGoal();
    client_arm_ik_->cancelGoal();
    client_base_->cancelGoal();
    client_arm_trajectory_->cancelGoal();
    unsubscribe();
    resetOdom(odom_dist_);
    resetOdom(odom_start_);
    resetOdom(odom_last_);
    resetOdom(odom_now_);
  }

  /*void resetStateSuccess(){
    //ROS_INFO("resetState");
    stopBase();
    initArm();
    is_arm_sent_goal_ = false;
    is_base_sent_goal_ = false;
    is_new_goal_ = true;

    time_start_ = ros::Time::now();
    task_.now = kArmStandbyHADY; //HADY
    //task_.now = kArmLookTable;
    client_arm_joints_->cancelGoal();
    client_arm_ik_->cancelGoal();
    client_base_->cancelGoal();
    client_arm_trajectory_->cancelGoal();
    unsubscribe();
    resetOdom(odom_dist_);
    resetOdom(odom_start_);
    resetOdom(odom_last_);
    resetOdom(odom_now_);
  }*/

   bool initSub(){
    subOdom();
    subBeam();
    subObject();

    boost::mutex::scoped_lock lock(odom_mutex_);
    ros::Time now = ros::Time::now();
    double time_diff = ( now - odom_msg_.header.stamp ).toSec();
    if( time_diff > 0.5 ){ 
      //ROS_WARN("Odom msg is older %.3lf than now",time_diff);
      return false;
    }
    double roll, pitch, yaw;
    aricc_utils::quat2Euler(odom_msg_.pose.pose.orientation, roll, pitch, yaw);
    odom_now_.x = odom_msg_.pose.pose.position.x;
    odom_now_.y = odom_msg_.pose.pose.position.y;
    odom_now_.theta = yaw;

    odom_start_  = odom_now_;
    odom_last_   = odom_now_;
    time_start_  = ros::Time::now();
    //ROS_WARN("Initial Odom");
    return true;
  }

   void rotatePose2D(double &x, double &y, double theta){
    Eigen::Vector3f src;
    Eigen::Vector3f dst;
    Eigen::Matrix3f rot_matrix_z;
    src << x,y,0;
    rot_matrix_z << cos(theta), -sin(theta), 0,
                    sin(theta), cos(theta), 0,
                    0, 0, 1;
    dst = rot_matrix_z * src;
    x = dst[0];
    y = dst[1];
  }

  void resetOdom(geometry_msgs::Pose2D& odom){
    odom.x = 0.0;
    odom.y = 0.0;
    odom.theta = 0.0;
  }

   bool updateOdom(){
     boost::mutex::scoped_lock lock(odom_mutex_);
     ros::Time now = ros::Time::now();
     double time_diff = ( now - odom_msg_.header.stamp ).toSec();
    if( time_diff > 0.5 ){                               
      //ROS_WARN("Odom msg is older %.3lf than now",time_diff);
      return false;
    }
      double roll, pitch, yaw;
      aricc_utils::quat2Euler(odom_msg_.pose.pose.orientation, roll, pitch, yaw);
      odom_now_.x = odom_msg_.pose.pose.position.x;
      odom_now_.y = odom_msg_.pose.pose.position.y;
      odom_now_.theta = yaw;

      double diff_x, diff_y, diff_th;
      diff_x  = odom_now_.x - odom_last_.x;
      diff_y  = odom_now_.y - odom_last_.y;
      diff_th = odom_now_.theta - odom_last_.theta;

    //odom_dist_.x = odom_now_.x - odom_start_.x;
    //odom_dist_.y = odom_now_.y - odom_start_.y;

    rotatePose2D( diff_x, diff_y, -odom_start_.theta );
    if(diff_x < 1.0) odom_dist_.x += diff_x;
    if(diff_y < 1.0) odom_dist_.y += diff_y;
    if(fabs(diff_th) < M_PI) odom_dist_.theta += diff_th;
    if(fabs(odom_dist_.theta) > M_PI*2.0) odom_dist_.theta = 0.0;
    //ROS_INFO( "odom:(%.3lf,%.3lf,%.3lf)",
    //odom_dist_.x, odom_dist_.y, aricc_utils::rad2Deg(odom_dist_.theta));

    odom_last_ = odom_now_;
    dist_left_ = dist_- odom_dist_.y;
    //ROS_INFO("dist left:%.3lf",dist_left_);
    return true;
  }

  StateEnum isGoalValid( youbot_unload_object_table::UnloadGoal goal){
    ROS_INFO("%s: accept goal", action_name_.c_str());

    if( isnan(goal.timeout) || isinf(goal.timeout)){
      ROS_ERROR("Goal has NAN item");
      return ABORTED;
    }
    size_t item_size = goal.items.size();
    size_t container_size = goal.containers.size();
    size_t tray_size = goal.tray_state.size();

    if( item_size < 0 || item_size > 3 ){
      ROS_ERROR("item size is %lu, but it should be within [0,3]", item_size);
      return ABORTED;
    }
    if( container_size < 0 || container_size > 3 ){
      ROS_ERROR("container size is %lu, but it should be within [0,3]", item_size);
      return ABORTED;
    }

    if( tray_size < 0  || tray_size > 3 ){
      ROS_ERROR("tray size is %lu, but it should be within [0,3]", tray_size);
      return ABORTED;
    }

    if(item_size != container_size) {
      ROS_ERROR("Item size should be same as container size");
      return ABORTED;
    }

    std::string str_goal = "Goal:";
    std::ostringstream strs;
    strs << goal.timeout;
    str_goal += strs.str();
    str_goal += " items:[";
    for(size_t i = 0; i < item_size; ++i){
      str_goal += goal.items[i];
      str_goal += " ";
    }
    str_goal += "],container:[";
    for(size_t i = 0; i < container_size; ++i){
      str_goal += goal.containers[i];
      str_goal += " ";
    }
    str_goal += "],tray:[";
    for(size_t i = 0; i < tray_size; ++i){
      str_goal += goal.tray_state[i];
      str_goal += " ";
    }
    str_goal += "]";
    ROS_INFO("%s", str_goal.c_str());

    timeout_ = goal.timeout;
    tray_state_ = goal.tray_state;
    if( tray_size == 0 ) return SUCCEEDED;
    if( item_size == 0 ) return SUCCEEDED;
    if( container_size == 0 ) return SUCCEEDED;
    
    for(size_t i = 0; i < table_list_.size(); ++i){
      double diff = 
        fabs(goal.table_height - table_list_[i].height);
      //ROS_INFO("ASAD == DIFF VALUE IS: %f, goal.height is: %f, table.height is: %f", diff, goal.table_height, table_list_[i].height);
      if(diff <= 0.0001){
        offset_x_ = table_list_[i].offset_x;
        offset_y_ = table_list_[i].offset_y;
        offset_z_ = table_list_[i].offset_z;
        object_detect_z_ = table_list_[i].object_detect_z;
        ROS_INFO("Got table params: %.3lf, %.3lf, %.3lf, %.3lf",
          offset_x_, offset_y_, offset_z_, object_detect_z_);
        break;
      }
    }
   
    if( changeTableHeightParam(table_param_namespace_1_,
          param_name_, object_detect_z_ ) == ABORTED)
      return ABORTED; 
    if( changeTableHeightParam(table_param_namespace_2_, 
          param_name_, object_detect_z_ ) == ABORTED)
      return ABORTED;
    if( changeTableHeightParam(container_1_param_namespace_, 
          param_name_, object_detect_z_ ) == ABORTED)
      return ABORTED;
    if( changeTableHeightParam(container_2_param_namespace_, 
          param_name_, object_detect_z_ ) == ABORTED)
      return ABORTED;

    std::vector<std::string> temp_tray  = goal.tray_state;
    std::vector< std::pair<std::string, std::string> > temp_items;
    temp_items.clear();

    for(unsigned int i = 0; i < goal.items.size(); ++i){
      //ROS_INFO("%d/%lu", i, goal.items.size());
      temp_items.push_back( std::make_pair(goal.items.at(i), goal.containers.at(i)) );
    }
    //printItemsAndTray(temp_items, tray_state_);

    task_.items.clear();
    std::vector< std::pair< std::string, std::string > >::iterator it_item;
    std::vector< std::string >::iterator it_tray;
    it_item = temp_items.begin();

    for( ; it_item != temp_items.end(); ++it_item ){
      //ROS_INFO("item now:%s", it_item->first.c_str());
      it_tray = temp_tray.begin();
      for(; it_tray != temp_tray.end(); ){
        //ROS_INFO("tray now:%s", it_tray->c_str());
        if(it_item->first == *it_tray){
          //ROS_INFO("Got item:%s", it_item->c_str());
          task_.items.push_back(*it_item );
          it_tray = temp_tray.erase(it_tray);
          //ROS_INFO("Break_0");
          break;
        }
        //ROS_INFO("Break_1");
        ++it_tray;
      }
    }
    
    ROS_INFO("Final task items:");
    printItemsAndTray(task_.items, tray_state_);
    //return ABORTED;
    if(task_.items.size() == 0) return SUCCEEDED;
    return ACTIVE;
  }
  
  StateEnum changeTableHeightParam(std::string detect_namespace,
    std::string param_name, double new_param ){
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;
    double_param.name =  param_name;
    double_param.value = new_param;
    conf.doubles.push_back(double_param);
    srv_req.config = conf;

    ros::NodeHandle target_nh = ros::NodeHandle(detect_namespace);
    std::string service_name = detect_namespace + "/set_parameters";
    unsigned int cnt = 0;
    double new_value = 0;

    while(1){
      try {
        ros::service::call(service_name, srv_req, srv_resp);
        ROS_WARN("Setting %s/%s: %.3lf", 
          target_nh.getNamespace().c_str(),
          param_name.c_str(), new_param );
      }
      catch(...) {
        ROS_ERROR("Something went wrong in the service call to dynamic_reconfigure");
        return ABORTED;
      }

      if(!target_nh.getParam( param_name, new_value)){
        ROS_ERROR("The %s, does not have parameter %s", 
        target_nh.getNamespace().c_str(), 
        param_name.c_str());
        return ABORTED;
      }
      if( new_value == new_param){
        ROS_INFO("The %s, have parameter %s, new value is %.3lf",
        target_nh.getNamespace().c_str(), 
        param_name.c_str(), new_value);
        break;
      }
      if(++cnt == 5) {
        ROS_ERROR("The %s, have parameter %s, current value is %.3lf, is not %.3lf", 
        target_nh.getNamespace().c_str(), param_name.c_str(), 
        new_value, new_param);
        return ABORTED;
      }
      sleep(1);
    }
  }

  void printItemsAndTray( 
    std::vector< std::pair<std::string, std::string> > items, 
    std::vector<std::string> tray ){
    std::vector< std::pair< std::string ,std::string > >::iterator it;
    //ROS_INFO("%lu",items.size());
    std::string str = "Item:[";
    for (it = items.begin(); it!=items.end(); ++it){
      str += it->first;
      str += ", ";
      str += it->second;
      str += " ";
    }
    str += "]";
    str += "Tray_state:[";
    for(size_t i = 0; i < tray.size(); ++i){
      str += tray.at(i);
      str += " ";
    }
    str += "]";
    ROS_INFO("%s",str.c_str()); 
  }

  void setAborted(youbot_unload_object_table::UnloadResult result, 
    std::string str = ""){
    ROS_ERROR("%s: Aborted, %s", action_name_.c_str(), str.c_str());
    result.tray_state = tray_state_;
    action_.setAborted(result);
    resetState();
  }

  void setSucceeded(youbot_unload_object_table::UnloadResult result){
    ROS_INFO(ANSI_COLOR_GREEN"%s: Succeeded, take %.3lf s"ANSI_COLOR_RESET, action_name_.c_str(),
      (time_now_ - time_start_).toSec());
    result.tray_state = tray_state_;
    action_.setSucceeded(result);
    resetState();
    //resetStateSuccess();
  }

  void setPreempted(){
    ROS_WARN("%s: Preempted", action_name_.c_str());
    action_.setPreempted();
    resetState();
  }

  void executeCB(const youbot_unload_object_table::UnloadGoalConstPtr &goal_msg){
    goal_ = *goal_msg;
    StateEnum goal_state = isGoalValid(goal_);
    if( goal_state == ABORTED ){
      setAborted(result_,"Goal is invalid");
      return;
    }
    else if( goal_state == SUCCEEDED){
      setSucceeded(result_);
      return;
    }
    resetState();
    ros::Rate r(rate_);
    
    while(action_.isActive() || ros::ok()){
      r.sleep();
      if( r.cycleTime() > ros::Duration(1.0/rate_) )
        ROS_WARN("%s: Control desired rate of %.3lfHz... the loop actually took %.4lf seconds", action_name_.c_str(), rate_, r.cycleTime().toSec());

      if( is_new_goal_ ) {
        if( initSub() ) is_new_goal_ = false;
        continue;
      }

      time_now_ = ros::Time::now(); 
      if( (time_now_ - time_start_).toSec() >= goal_.timeout && 
        goal_.timeout != 0.0 ){
        setAborted(result_, "Timeout");
        return;
      }

      if( action_.isPreemptRequested() ){
        if( action_.isNewGoalAvailable() ){
          goal_ = *(action_.acceptNewGoal());
          StateEnum goal_state = isGoalValid(goal_);
          if( goal_state == ABORTED ){
            setAborted(result_,"Goal is invalid");
            return;
          }
          else if( goal_state == SUCCEEDED){
            setSucceeded(result_);
            return;
          }
          else if(goal_state = ACTIVE) {
            ROS_INFO("%s: accept new goal", action_name_.c_str());
            resetState();
          }
        }
        else{ setPreempted(); return; }
      }

      //control loop
      updateOdom();
      StateEnum state = taskStep();
      if( state == SUCCEEDED ) { 
        setSucceeded(result_); return; }
      else if( state == ABORTED ) { 
       setAborted(result_); return; }
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "youbot_unload_object_table_server");
  UnloadAction action(ros::this_node::getName());
  if(!action.initAction()) return 0;
  ros::spin();
  return 0;
}
