//edited by khaliq 22/7/2019
#include <string>

/*
  TaskEnum{}
  Assign tasks

  class Task{}
  switch(now): mapping task states (variable to string)

  class LoadAction{}
  public: type definitions + StateEnum

  protected: Variables
  ros
  boost(multithreader)
  laser
  state flags
  armik
  trajectory

  Switch(task_.now)        PLEASE UPDATE THIS
  kArmLookTable
  kForwardAlignZero,
  kBackwardAlignZero,
  kArmMidPointBeforeUnload,
  kStopBase,
  kArmUnload,
  kOpenGripperHalfClose,
  kOpenGripperFully,
  kCloseGripperUnload,
  kCheckArmIkBeforeGrasp,
  kArmReadyAfterUnload,
  kArmReadyBeforeLoadTable,
  kCheckState,
  kBaseAlignObject,
  kArmLoadTable,
  kOpenGripperLoadTable,
  kStaticFindObject,
  kBaseMoveLeft,
  kBaseMoveRight,
  kFinishTask,
  kUpdateBestList,
  kArmStandby,
  kEmpty

*/

enum StateEnum {ACTIVE = 0, SUCCEDDED, ABORTED};

bool loadTables(); //table height, offset values from stations.yaml

bool initAction(); //checking if all the package dependancies are up

//Subsrcibing and unsubscribing to rostopics start:
void subObject();

void subOdom();

void subBeam();

void unsubObject();

void unsubOdom();

void unsubBeam();

void unsubscribe(); //unsubs to all

//end

bool loadAllArmJoints(); //Maps joints data to Joints Variable UNUSED

bool loadArmJoints(); //Loads joints data from joints.yaml UNUSED

bool loadAllArmTrajectories(); //Maps trajectory data to trajectory Variable

bool loadArmTrajectory(); //Loads trajectory data from arm_trajectory.yaml

void objectCB(const aricc_vision_msgs::ObjectArray::ConstPtr& msg); //Pushes back object_msgs

void odomCB(const nav_msgs::Odometry::ConstPtr& msg); //assigns odom_msgs_ data

void beamCB(const aricc_laser_pipeline::BeamDistArray::ConstPtr& msg); //assigns beam_msgs_ data

StateEnum moveGripper(std::string cmd); //receive gripper commands and moves gripper according to joint StateEnum

StateEnum initArm(); //Initializes arm to a set postion i.e. setting to nav_pose at the start

StateEnum moveArmTrajectory(Trajectory traj); //receive commands and moves the joints according to duration set in arm_trajectory.yaml

StateEnum moveArm(std::string cmd); //Receive commands and moves by joints. Without .yaml UNUSED

StateEnum moveArmJoints(Trajectory traj); //receive commands and moves by Joints
//To note: maps trajectory to joints. using arm_trajectory.yaml

StateEnum moveArmIK(geometry_msgs::Point pos, geometry_msgs::Vector3 ori,
    bool move_arm = true); //Moves arm down according to moveArmUpDownIk data

StateEnum moveArmUpDownIk(aricc_vision_msgs::Object object, int dir,
    bool move_arm = true);
// Finds table or container to unload on

StateEnum stopBase(); //Sets the movements to zero. Thus, stopping the base

StateEnum moveBase( geometry_msgs::Pose2D pose); //Gets movement, moves the robot. (Positional: current base pose is considered 0,0,0)

StateEnum findItemAndContainer(
  std::vector< std::pair<std::string, std::string> > items,
  std::vector< std::pair<std::string, std::string> >& best_items,
  std::vector< aricc_vision_msgs::Object> & best_containers ){
    boost::mutex::scoped_lock lock(object_mutex_);
//Finds the coordinates for table and container

void updateItemListAndTray(
          std::vector<std::string>& items,
          std::vector<std::string>& tray,
          aricc_vision_msgs::Object object,
          size_t slot); //updates item list and tray states

size_t getEmptyTraySlot( std::vector<std::string> tray, size_t& slot );
//finds empty slot on the tray

bool isBaseMoveEnoughRight(double dist = 0); //checks for obstacles on the right

bool isBaseMoveEnoughLeft(double dist = 0); //checks for obstacles on the left

StateEnum taskStep(); //State machine programming

void resetState(); //resets all check flags and unsubscribes

bool initSub(); //subscribe to topics and sets odom positions

void resetOdom(geometry_msgs::Pose2D& odom); //reset odom coordinates

void rotatePose2D(double &x, double &y, double theta); //shifts the robot rotation

bool updateOdom(); //updates odometry positions

StateEnum isGoalValid( youbot_load_object::LoadGoal goal);

StateEnum changeTableHeightParam(std::string detect_namespace,
  std::string param_name, double new_param );

StateEnum changeObjectsParam(std::string detect_namespace,
  std::string param_name, std::string new_param );

void setAborted(youbot_load_object::LoadResult result,
  std::string str = "");

void setSucceeded(youbot_load_object::LoadResult result);

void setPreempted();

void executeCB(const youbot_load_object::LoadGoalConstPtr &goal_msg);
