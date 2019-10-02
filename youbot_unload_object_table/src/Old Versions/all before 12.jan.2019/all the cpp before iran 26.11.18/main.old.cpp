#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <youbot_unload_object_table/UnloadAction.h>
//#include <youbot_unload_object_table/ServerConfig.h>

#include <youbot_base_local_move/BaseAction.h>
#include <youbot_arm_joints/JointAction.h>
#include <youbot_arm_ik/IkAction.h>

#include <geometry_msgs/Pose2D.h>
#include <aricc_utils/geometry_utils.h>
#include <aricc_vision_msgs/Object.h>
#include <aricc_vision_msgs/ObjectArray.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <utility>
#include <iostream>
#include <assert.h>

enum TaskEnum { kArmStandby = 10,
                kOpenGripperBeforeUnload,
                kArmReadyBeforeUnload,
                kArmReadyBeforeUnloadCenter,
                kArmUnload,
                kCloseGripperUnload,
                kArmReadyAfterUnloadCenter,
                kArmReadyAfterUnload,
                kArmReadyBeforeLoadTable,
                kArmLoadTable,
                kOpenGripperLoadTable,
                kArmReadyAfterLoadTable,
                kBaseMoveLeft,
                kFinishTask,
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
  std::vector<std::string> items;
  
  Task(): now(kArmStandby), last(kEmpty){
    items.clear();
  }
  
  ~Task(){}

  std::string toString() const{
    switch(now){
      case kArmStandby:                return "kArmStandby";
      case kOpenGripperBeforeUnload:   return "kOpenGripperBeforeUnload";
      case kArmReadyBeforeUnload:      return "kArmReadyBeforeUnload";
      case kArmReadyBeforeUnloadCenter:return "kArmReadyBeforeUnloadCenter";
      case kArmUnload:                 return "kArmUnload";
      case kCloseGripperUnload:        return "kCloseGripperUnload";
      case kArmReadyAfterUnload:       return "kArmReadyAfterUnload";
      case kArmReadyAfterUnloadCenter: return "kArmReadyAfterUnloadCenter";
      case kArmReadyBeforeLoadTable:   return "kArmReadyBeforeLoadTable";
      case kArmLoadTable:              return "kArmLoadTable";
      case kOpenGripperLoadTable:      return "kOpenGripperLoadTable";
      case kArmReadyAfterLoadTable:    return "kArmReadyAfterLoadTable";
      case kBaseMoveLeft:              return "kBaseMoveLeft";
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
  typedef std::vector<trajectory_msgs::JointTrajectoryPoint> Trajectory;

  //typedef youbot_unload_object_table::ServerConfig Config;
  enum StateEnum { ACTIVE = 0, SUCCEEDED, ABORTED };

protected:
  ros::NodeHandle nh_, pnh_;
  actionlib::SimpleActionServer
    <youbot_unload_object_table::UnloadAction> action_;
  std::string action_name_;
  //boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
  boost::shared_ptr<AcArmJoints> client_arm_joints_;
  boost::shared_ptr<AcBase>      client_base_;
  boost::shared_ptr<AcArmTrajectory> client_arm_trajectory_;
  boost::shared_ptr<AcArmIk>         client_arm_ik_; 

  boost::mutex mutex_;
  std::vector<std::string> tray_state_;
  Task task_;
  std::string table_type_;
  std::string container_colour_;
  aricc_vision_msgs::Object best_object_;
  size_t best_slot_;

  bool is_arm_sent_goal_;
  bool is_base_sent_goal_;

  double rate_;
  double timeout_;
  double offset_x_;
  double offset_y_;
  double offset_z_;
  double object_detect_z_;

  Trajectory armTrajectoryLookTable_;
  Trajectory armTrajectoryStandby_;
  Trajectory armTrajectoryGoTray_1_;
  Trajectory armTrajectoryGoTray_2_;
  Trajectory armTrajectoryGoTray_3_;
  Trajectory armTrajectoryLeaveTray_1_;
  Trajectory armTrajectoryLeaveTray_2_;
  Trajectory armTrajectoryLeaveTray_3_;


  std::vector<double> gripperOpen_; 
  std::vector<double> gripperClose_; 
  
  std::vector<Table> table_list_;

  youbot_unload_object_table::UnloadGoal goal_;
  youbot_unload_object_table::UnloadFeedback feedback_;
  youbot_unload_object_table::UnloadResult result_;
 
  ros::Time time_now_, time_start_;
 
public:
  UnloadAction(std::string name) :
    pnh_("~"),is_arm_sent_goal_(false),is_base_sent_goal_(false),
    action_(nh_, name, boost::bind(&UnloadAction::executeCB, this, _1), false),
    action_name_(name){
  }

  ~UnloadAction(void){
  }

  bool loadTables(){
    table_list_.clear();
    //Loading tasks from config files
    ROS_INFO("----------");
    ROS_INFO("Loading tables ...");
    XmlRpc::XmlRpcValue table_list;

    if( pnh_.getParam("tables", table_list) ){
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
    
    pnh_.param<std::string>
      ("object_topic", object_topic_, "detected_objects");
    pnh_.param<std::string>
      ("odom_topic", odom_topic_, "odom");
    std::string object_detect_namespace;
    pnh_.param("object_detect_namespace", object_detect_namespace, std::string("/youbot_object_detection/RotatedRectFinder/"));
    pnh_.param("table_height_param", table_height_param_, std::string("z"));
    pnh_.param("service_name", service_name_, std::string("/youbot_2d_vision/object_detect_RotatedRectFinder/set_parameters"));
    
    target_nh_ = ros::NodeHandle(object_detect_namespace);
    dynamic_reconfigure_service_ = target_nh_.serviceClient<dynamic_reconfigure::Reconfigure>("set_parameters", true);


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

    detected_objects_.clear();
    action_.start();
    ROS_INFO("Starting %s ...", action_name_.c_str());
    return true;
  }

  void subObject(){
    sub_object_ = nh_.subscribe( object_topic_,1, &LoadAction::objectCB,this);
  }

  void subOdom(){
    sub_odom_   = nh_.subscribe( odom_topic_,  1, &LoadAction::odomCB,this);
  }

  void unsubObject(){
    sub_object_.shutdown();
  }

  void unsubOdom(){
    sub_odom_.shutdown();
  }

  void unsubscribe(){
    unsubObject();
    unsubOdom();
  }

  bool loadAllArmTrajectories(){
    if(!loadArmTrajectory("arm_trajectory_look_table",armTrajectoryLookTable_))
      return false;
    if(!loadArmTrajectory("arm_trajectory_standby",armTrajectoryStandby_))
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
    return true;
  }


  //void configCallback(Config &config, uint32_t level){
  //  boost::mutex::scoped_lock lock(mutex_);
  //}

  bool loadAllArmJoints(){
    if(!loadArmJoints("arm_joints_unload_table",armJointsUnloadTable_))
      return false;
    if(!loadArmJoints("arm_joints_standby",armJointsStandby_))
      return false;
    if(!loadArmJoints("arm_joints_ready_tray_left",armJointsReadyTrayLeft_))
      return false;
    if(!loadArmJoints("arm_joints_ready_tray_center",armJointsReadyTrayCenter_))
      return false;
    if(!loadArmJoints("arm_joints_ready_tray_right",armJointsReadyTrayRight_))
      return false;
    if(!loadArmJoints("arm_joints_tray_left",armJointsTrayLeft_))
      return false;
    if(!loadArmJoints("arm_joints_tray_center",armJointsTrayCenter_))
      return false;
    if(!loadArmJoints("arm_joints_tray_right",armJointsTrayRight_))
      return false;
    if(!loadArmJoints("arm_joints_transport",armJointsTransport_))
      return false;
    if(!loadArmJoints("arm_joints_before_tray_center",armJointsBeforeTrayCenter_))
      return false;
    //if(!loadArmJoints("gripper_open",gripperOpen_))
    //  return false;
    //if(!loadArmJoints("gripper_close",gripperClose_))
    //  return false;
    return true;
  }

  bool loadArmJoints(std::string name, std::vector<double>& joints){
    //ROS_INFO("Loading arm joints ...");
    try{
      XmlRpc::XmlRpcValue list;
      if( !pnh_.hasParam(name) ){
        ROS_ERROR("Cannot find %s", name.c_str());
        return false;
      }
      pnh_.getParam( name, list );
      ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      //ROS_INFO("Found list size:%d", list.size());
      joints.clear();
      std::string str = "Loaded ";
      str += name;
      str +=":[";
      for(size_t i = 0; i < list.size(); ++i){
        ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        std::string value = static_cast<std::string>(list[i]);
        str += value;
        str += " ";
        joints.push_back( atof(value.c_str()) );
      }
      str += "]";
      ROS_INFO("%s", str.c_str());
    }
    catch(ros::Exception e){
      ROS_ERROR("%s", e.what());
      return false;
    }
    return true;
  }

  StateEnum moveGripper( std::string cmd ){
    if(!is_arm_sent_goal_){
      youbot_arm_joints::JointGoal goal;
      std::vector<double> joints;
      joints.resize(2);
      if(cmd == "open"){
        joints[0] = 0.012;
        joints[1] = 0.012;
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
    youbot_arm_joints::JointGoal goal;
    goal.timeout = 20;
    goal.name = "arm";
    goal.joints = armJointsTransport_;
    client_arm_joints_->sendGoal(goal);
    client_arm_joints_->waitForResult(ros::Duration(20));
    if( client_arm_joints_->getState().toString() == "SUCCEEDED" ){
      is_arm_sent_goal_ = false;
      return SUCCEEDED;
    }
    else if( client_arm_joints_->getState().toString() == "ABORTED" ){
      is_arm_sent_goal_ = false;
      return ABORTED;
    }
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

/*
  StateEnum moveArmIk( geometry_msgs::Point pos, geometry_msgs::Vector3 ori,
    bool move_arm = true ){
    if(!is_arm_sent_goal_){
      youbot_arm_ik::IkGoal goal;
      goal.timeout = 20;
      goal.position = pos;
      goal.orientation = ori;
      goal.name = "preferred_pitch_ik";
      goal.pitch = M_PI_2;
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

  StateEnum moveArmUpDownIk(aricc_vision_msgs::Object object, int dir,
    bool move_arm = true){
    StateEnum state;
    geometry_msgs::Point pos = object.position;
    pos.x += offset_x_;
    pos.y += offset_y_;
    geometry_msgs::Vector3 ori;
    ori.x = object.orientation.z + offset_roll_;
    ori.y = M_PI_2;
    ori.z = 0.0;
    if(fabs(pos.y) >= 0.05) return ABORTED;

    //move arm up
    if(dir == 1) {
      pos.z = offset_z_ + 0.04;
      state = moveArmIk(pos, ori, move_arm);
      if(move_arm && state == SUCCEEDED) sleep(1);
      return state;
    }
    //move arm down
    if(dir == -1){
      //ROS_INFO("%u",cnt_arm_move_);
      double offset = cnt_arm_move_*0.04;
      pos.z = offset_z_ + 0.04 - offset;
      state = moveArmIk(pos, ori, move_arm);
      if(state == SUCCEEDED) cnt_arm_move_++;
      else if(state == ABORTED) return state;

      if(cnt_arm_move_ == 2){
        cnt_arm_move_ = 0;
        if(move_arm && state == SUCCEEDED) sleep(1);
        return state;
      }
      return ACTIVE;
    }
  }
  */
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

  void updateItemListAndTray(std::vector<std::string>& items,
    std::vector<std::string>& tray, size_t slot){
    tray[slot] = "empty";
    if(items.size() != 0) items.erase(items.begin());
    std::string str = "Item_state:[";
    for(size_t i = 0; i < items.size(); ++i){
      str += items.at(i);
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

  size_t getTraySlot( std::string item, std::vector<std::string> tray ){
    for( size_t i = 0; i < tray.size(); ++i ){
      if(tray.at(i) == item) return i;
    }
  }

  StateEnum taskStep(){
    if( task_.now != task_.last )
      ROS_INFO( "Task now: %s",task_.toString().c_str() );
    task_.last = task_.now;

    StateEnum state;
    geometry_msgs::Pose2D basePose;
    switch(task_.now){
      case kArmStandby:
        state = moveArmJoints(armJointsStandby_);
        if(state == SUCCEEDED) task_.now = kOpenGripperBeforeUnload;
        //else if(state == ABORTED) return ABORTED;
      break;

      case kOpenGripperBeforeUnload:
        state = moveGripper("open");
        if(state == SUCCEEDED){ 
          task_.now = kArmReadyBeforeUnload;
          best_slot_ = getTraySlot( task_.items[0], tray_state_);
        }
        //else if(state == ABORTED) task_.now = kArmLookTable;
      break;
      
      case kArmReadyBeforeUnload:
        if(best_slot_ == 0) 
          state = moveArmJoints(armJointsReadyTrayLeft_);
        if(best_slot_ == 1) 
          state = moveArmJoints(armJointsReadyTrayCenter_);
        if(best_slot_ == 2) 
          state = moveArmJoints(armJointsReadyTrayRight_);
        if(state == SUCCEEDED){
          if(best_slot_ == 1) task_.now = kArmReadyBeforeUnloadCenter;
          else task_.now = kArmUnload;
        }
        //else if(state == ABORTED) task_.now = kArmLookTable;
      break;

      case kArmReadyBeforeUnloadCenter:
        state = moveArmJoints(armJointsBeforeTrayCenter_);
        if(state == SUCCEEDED) task_.now = kArmUnload;
      break;

      case kArmUnload:
        if(best_slot_ == 0) 
          state = moveArmJoints(armJointsTrayLeft_);
        if(best_slot_ == 1) 
          state = moveArmJoints(armJointsTrayCenter_);
        if(best_slot_ == 2) 
          state = moveArmJoints(armJointsTrayRight_);
        if(state == SUCCEEDED) task_.now = kCloseGripperUnload;
        //else if(state == ABORTED) return ABORTED;
      break;

      case kCloseGripperUnload:
        if(task_.items[0] == "S40_40_G" ||
           task_.items[0] == "S40_40_B" ||
           task_.items[0] == "M30")
          state = moveGripper("close_light");
        else state = moveGripper("close_heavy");
        if(state == SUCCEEDED) {
          if(best_slot_ == 1) task_.now = kArmReadyAfterUnloadCenter;
          else task_.now = kArmReadyAfterUnload;
        }
        //else if(state == ABORTED) return ABORTED;
      break;

      case kArmReadyAfterUnloadCenter:
        state = moveArmJoints(armJointsBeforeTrayCenter_);
        if(state == SUCCEEDED) task_.now = kArmReadyAfterUnload;
      break;
      
      case kArmReadyAfterUnload:
        if(best_slot_ == 0) 
          state = moveArmJoints(armJointsReadyTrayLeft_);
        if(best_slot_ == 1) 
          state = moveArmJoints(armJointsReadyTrayCenter_);
        if(best_slot_ == 2) 
          state = moveArmJoints(armJointsReadyTrayRight_);
        if(state == SUCCEEDED) task_.now = kArmReadyBeforeLoadTable;
        //else if(state == ABORTED) return ABORTED;
      break;

      case kArmReadyBeforeLoadTable:
        state = moveArmJoints(armJointsStandby_);
        if(state == SUCCEEDED) task_.now = kArmLoadTable;
        //else if(state == ABORTED) return ABORTED;
      break;

      case kArmLoadTable:
        state = moveArmJoints(armJointsUnloadTable_);
        if(state == SUCCEEDED) task_.now = kOpenGripperLoadTable;
        //else if(state == ABORTED) return ABORTED;
      break;

      case kOpenGripperLoadTable:
        state = moveGripper("open");
        if(state == SUCCEEDED) task_.now = kArmReadyAfterLoadTable;
        //else if(state == ABORTED) return ABORTED;
      break;

      case kArmReadyAfterLoadTable:
        state = moveArmJoints(armJointsStandby_);
        if(state == SUCCEEDED){ 
          updateItemListAndTray( task_.items, tray_state_, best_slot_);
          if(task_.items.size() == 0) task_.now = kFinishTask;
          else task_.now = kBaseMoveLeft;
        }
        //else if(state == ABORTED) return ABORTED;
      break;

      case kBaseMoveLeft:
        basePose.x = 0.0; basePose.y = 0.1; basePose.theta = 0.0;
        state = moveBase(basePose);
        if(state == SUCCEEDED) task_.now = kArmStandby;
      break;

      case kFinishTask:
        state = moveArmJoints(armJointsTransport_);
        if(state == SUCCEEDED) return SUCCEEDED;
        else if(state == ABORTED) return ABORTED;
    }
    return ACTIVE;
  }

  void resetState(){
    stopBase();
    initArm();
    is_arm_sent_goal_ = false;
    is_base_sent_goal_ = false;

    time_start_ = ros::Time::now();
    task_.now = kArmStandby;
    client_arm_joints_->cancelGoal();
    client_base_->cancelGoal();
  }

  StateEnum isGoalValid( youbot_unload_object_table::UnloadGoal goal){
    ROS_INFO("%s: accept goal", action_name_.c_str());

    if( isnan(goal.timeout) || isinf(goal.timeout)){
      ROS_ERROR("Goal has NAN item");
      return ABORTED;
    }
    /*
    if( goal.table_type != "table" ||
        goal.table_type != "rotate_table" || 
        goal.table_type != "shelf" ||
        goal.table_type != "container"){
      ROS_ERROR("Goal has wrong table type");
      return ABORTED;
    }
    if(goal.container_colour != "red" ||
       goal.container_colour != "blue"){
      ROS_ERROR("Only support red and blue colour");
      return ABORTED
    }
    if( isnan(goal.table_height) || isinf(goal.table_height)){
      ROS_ERROR("Goal has NAN item");
      return ABORTED;
    }*/

    size_t item_size = goal.items.size();
    size_t tray_size = goal.tray_state.size();

    if( item_size < 0 || item_size > 3 ){
      ROS_ERROR("item size is %lu, but it should be within [0,3]", item_size);
      return ABORTED;
    }
    if( tray_size < 0  || tray_size > 3 ){
      ROS_ERROR("tray size is %lu, but it should be within [0,3]", tray_size);
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
    str_goal += "],tray:[";
    for(size_t i = 0; i < tray_size; ++i){
      str_goal += goal.tray_state[i];
      str_goal += " ";
    }
    str_goal += "]";
    ROS_INFO("%s", str_goal.c_str());

    timeout_ = goal.timeout;
    //table_type_ = goal.table_type;
    //container_colour_ = goal.container_colour;
    tray_state_ = goal.tray_state;
    if( tray_size == 0 ) return SUCCEEDED;
    if( item_size == 0 ) return SUCCEEDED;
    
    /*
    for(size_t i = 0; i < table_list_.size(); ++i){
      if(goal.table_height == table_list_[i].height){
        offset_x_ = table_list_[i].offset_x;
        offset_y_ = table_list_[i].offset_y;
        offset_z_ = table_list_[i].offset_z;
        object_detect_z_ = table_list_[i].object_detect_z;
      }
    }*/

    std::vector<std::string> temp_tray  = goal.tray_state;
    std::vector<std::string> temp_items = goal.items;

    task_.items.clear();
    std::vector< std::string >::iterator it_item;
    std::vector< std::string >::iterator it_tray;
    it_item = temp_items.begin();
    for( ; it_item != temp_items.end(); ++it_item ){
     // ROS_INFO("item now:%s", it_item->c_str());
      it_tray = temp_tray.begin();
      for(; it_tray != temp_tray.end(); ){
        //ROS_INFO("tray now:%s", it_tray->c_str());
        if(*it_item == *it_tray){
          //ROS_INFO("Got item:%s", it_item->c_str());
          task_.items.push_back(*it_item);
          it_tray = temp_tray.erase(it_tray);
          break;
        }
        ++it_tray;
      }
    }
    
    std::string str = "Finalize task items: [";
    for(size_t i = 0; i < task_.items.size(); ++i){
      str += task_.items.at(i);
      str += " ";
    } 
    str += "]";
    ROS_INFO("%s", str.c_str());
    if(task_.items.size() == 0) return SUCCEEDED;
    return ACTIVE;
  }

  void setAborted(youbot_unload_object_table::UnloadResult result, 
    std::string str = ""){
    ROS_ERROR("%s: Aborted, %s", action_name_.c_str(), str.c_str());
    result.tray_state = tray_state_;
    action_.setAborted(result);
    resetState();
  }

  void setSucceeded(youbot_unload_object_table::UnloadResult result){
    ROS_INFO("%s: Succeeded, take %.3lf s", action_name_.c_str(),
      (time_now_ - time_start_).toSec());
    result.tray_state = tray_state_;
    action_.setSucceeded(result);
    resetState();
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
      StateEnum state = taskStep();

      if( state == SUCCEEDED ) { setSucceeded(result_); return; }
      else if( state == ABORTED ) { setAborted(result_); return; }
      
      r.sleep();
      if( r.cycleTime() > ros::Duration(1.0/rate_) )
        ROS_WARN("%s: Control desired rate of %.3lfHz... the loop actually took %.4lf seconds", action_name_.c_str(), rate_, r.cycleTime().toSec());
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
