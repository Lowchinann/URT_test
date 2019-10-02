#include <actionlib/client/simple_action_client.h>
#include "dynamic_reconfigure/server.h"
#include <youbot_unload_object_table/UnloadAction.h>
#include <youbot_unload_object_table/ControllerConfig.h>
#include <aricc_utils/geometry_utils.h>
#include <geometry_msgs/Pose2D.h>
#include <iostream>
#include <assert.h>
#include <ros/ros.h>

class YoubotUnloadObjectTableController{
  public:
    typedef actionlib::SimpleActionClient<youbot_unload_object_table::UnloadAction> ActionClient;
    typedef youbot_unload_object_table::ControllerConfig Config;

    YoubotUnloadObjectTableController():nh_("~"),rate_(5){
      client_.reset( new ActionClient("youbot_unload_object_table_server", true) );
      srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (nh_);
      dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind (&YoubotUnloadObjectTableController::configCallback, this, _1, _2);
      srv_->setCallback(f);
      nh_.param<double>("table_height", table_height_, 0.1);
      tray_state_.clear();
      tray_state_.resize(3);
      nh_.param<std::string>("tray_1", tray_state_[0], "empty");
      nh_.param<std::string>("tray_2", tray_state_[1], "empty");
      nh_.param<std::string>("tray_3", tray_state_[2], "empty");
      items_.clear();
      containers_.clear();

      std::string item, container;
      if(nh_.hasParam("item_1") && nh_.hasParam("container_1")){
        nh_.getParam("item_1", item);
        nh_.getParam("container_1", container);
        ROS_INFO("Add item:%s, container:%s",
          item.c_str(),container.c_str());
        items_.push_back(item);
        containers_.push_back(container);
      }
      if(nh_.hasParam("item_2") && nh_.hasParam("container_2")){
        nh_.getParam("item_2", item);
        nh_.getParam("container_2", container);
        ROS_INFO("Add item:%s, container:%s",
          item.c_str(),container.c_str());
        items_.push_back(item);
        containers_.push_back(container);
      }
      if(nh_.hasParam("item_3") && nh_.hasParam("container_3")){
        nh_.getParam("item_3", item);
        nh_.getParam("container_3", container);
        ROS_INFO("Add item:%s, container:%s",
          item.c_str(),container.c_str());
        items_.push_back(item);
        containers_.push_back(container);
      }
      if(!client_->waitForServer(ros::Duration(5.0))){
        ROS_ERROR("youbot_unload_object_table_server is not excuting");
      }
    }

    ~YoubotUnloadObjectTableController(){
    }

    void spin(){
      ros::Rate r(rate_);
      while( nh_.ok() ){
        ros::spinOnce();
        r.sleep();
      }
    }

  private:
    ros::NodeHandle nh_;
    unsigned int rate_;
    boost::shared_ptr<ActionClient> client_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    double timeout_;
    double table_height_;
    std::vector<std::string> items_;
    std::vector<std::string> containers_;
    std::vector<std::string> tray_state_;

    void configCallback(Config &config, uint32_t level){
      timeout_    = config.timeout;
      if(config.run) loadObject();
      config.run = false;
    }

    void printGoal(youbot_unload_object_table::UnloadGoal goal){
      std::string str_goal = "Goal:";
      std::ostringstream strs;
      strs << goal.timeout;
      str_goal += strs.str();
      str_goal += " items:[";
      for(size_t i = 0; i < goal.items.size(); ++i){
        str_goal += goal.items[i];
        str_goal += " ";
      }
      str_goal += "],container:[";
      for(size_t i = 0; i < goal.containers.size(); ++i){
        str_goal += goal.containers[i];
        str_goal += " ";
      }
      str_goal += "],tray:[";
      for(size_t i = 0; i < goal.tray_state.size(); ++i){
        str_goal += goal.tray_state[i];
        str_goal += " ";
      }
      str_goal += "]";
      ROS_INFO("%s", str_goal.c_str());
    }
    
    void loadObject(){
      youbot_unload_object_table::UnloadGoal goal;
      goal.timeout    = timeout_;
      goal.tray_state = tray_state_;
      goal.items      = items_;
      goal.containers = containers_;
      goal.table_height = table_height_;
      printGoal(goal);
      client_->sendGoal(goal);
      if(client_->waitForResult()){
        actionlib::SimpleClientGoalState state = client_->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        std::vector<std::string> tray;
        tray = client_->getResult()->tray_state;
        std::string str = "Tray:[";
        for(size_t i = 0; i < tray.size(); ++i){
          str += tray[i];
          str += " ";
        }
        str += "]";
        ROS_INFO("%s",str.c_str());
      }
    }
};

int main( int argc, char** argv ){
  ros::init( argc, argv, "youbot_unload_object_table_controller" );
  YoubotUnloadObjectTableController rt_;
  rt_.spin();
  return 0;
}
