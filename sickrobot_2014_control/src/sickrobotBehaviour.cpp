#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <hector_worldmodel_msgs/ObjectModel.h>
#include <hector_worldmodel_msgs/Object.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <sickrobot_2014_msgs/StartMission.h>
#include <sickrobot_2014_msgs/ExecuteState.h>

#include <math.h>

//#include <LinearMath/btTransform.h>
#include <tf_conversions/tf_eigen.h>

#include <tf/transform_listener.h>

#include <stdio.h>
#include <string>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//#include <sickrobot_vision/ballInGrabber.h>
//#include <sickrobot_msgs/SetBallColor.h>

// to run sm() as a thread
#include <boost/thread.hpp>

//#include <dynamixel_msgs/JointState.h>
#include <sensor_msgs/JointState.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class sickrobot_behaviour {
public:
  sickrobot_behaviour()
  {
    // instantiate publisher and subscriber
    ros::NodeHandle worldmodel(std::string("worldmodel"));
    ros::NodeHandle root("");
    cmdVelPublisher = root.advertise<geometry_msgs::Twist>("cmd_vel", 10, false);
    messagePublisher = root.advertise<std_msgs::String>("sickrobot/print", 10, false);

    //modelSubscriber = worldmodel.subscribe("objects", 10, &sickrobot_behaviour::objectsCb, this);


    ros::NodeHandle sick_nh(std::string("sickrobot"));
    startMission = sick_nh.advertiseService("start_mission", &sickrobot_behaviour::startMissionCb, this);
//    executeState = sick_nh.advertiseService("execute_state", &sickrobot_behaviour::executeStateCb, this);


    _run = false;
    _executeSingleState = false;
    _nextSingleState = STATE_IDLE;



    // get moveBaseClient
    // tell the action client that we want to spin a thread by default
    //mbClient = new MoveBaseClient ("move_base", true);
    // wait for the action server to come up
    //while (!mbClient->waitForServer(ros::Duration(5.0)))
      //ROS_INFO("Waiting for the move_base action server to come up");
     //
  }


  ~sickrobot_behaviour() {
    _state = STATE_STOP;

    //delete mbClient;
  }

  void run0()
  {
    ROS_INFO("I am a thread 0!");

    // initialize


    // simple test of all states
//    manipulator_test();

    // run statemachine
    sm();
  }

  void start0()
  {
    m_Thread0 = boost::thread(&sickrobot_behaviour::run0, this);
  }

  void join0()
  {
    m_Thread0.join();
  }

  void run1()
  {
    ROS_INFO("I am thread 1!");

    while (true) {
      if (_state == STATE_IDLE ||
         // _state == STATE_DRIVE_TO_GOAL ||
          //_state == STATE_ERROR ||
          _state == STATE_STOP
         ) {
        // do nothing
        ROS_INFO("do nothing");
      } else if (_state == STATE_DRIVE_1m){
        ROS_INFO("do something if STATE_DRIVE_1m");
          //ROS_INFO("run1: went into the state request for finding target balls");
//          if (getNearestBall()){
//            ROS_DEBUG("run1: Send mbClient command to cancel all Goals");
//            mbClient->cancelAllGoals();
//            ROS_DEBUG("run1: change state to approach ball automated");
//            setState(STATE_APPROACH_BALL_AUTOMATED);
//          }
      }
      ros::Duration(1.0).sleep();
    }
  }

  void start1()
  {
    m_Thread1 = boost::thread(&sickrobot_behaviour::run1, this);
  }

  void join1()
  {
    m_Thread1.join();
  }

protected:
  void sm()
  {
    ROS_INFO("sm started");
    _state = STATE_IDLE;
    _state_lock = false;


    while (_state != STATE_STOP) {
      // wait until start_mission is true
     // ROS_INFO("before while");
      while (!_run && !_executeSingleState)
        ros::Duration(0.1).sleep();

      if (_executeSingleState)
        _state = _nextSingleState;

      std_msgs::String msg;
    msg.data = getStateName(_state);
    messagePublisher.publish(msg);
    ROS_INFO("_state = %i (%s)", _state, getStateName(_state).c_str());

      switch(_state) {          
        case STATE_START:
          start_task();
          break;

        case STATE_STOP:
          _state_lock = false;
          break;

        case STATE_DRIVE_1m:
         // driveToGoal();
          drive_1m();
          break;
      };

//      if (_executeSingleState) {
//        _executeSingleState = false;
//        _state_lock = false;
//        _state = STATE_START;
//      }
    }

    ROS_INFO("sm exited with state %i", _state);
  }

  void start_task()
  {


    ROS_INFO("state: start");

    _state = STATE_DRIVE_1m;

  }

  void drive_1m()
  {
    ROS_INFO("state: drive_1m");
    _state = STATE_STOP;
  }

  int curr_waypoint_index;
  std::vector<geometry_msgs::Point> waypoints;




//  bool adjustYaw(const geometry_msgs::Point& targetPosMap, double angular_threshold)
//  {
//    // initialize Twist-Message
//    geometry_msgs::Twist cmdVelTwist;
//    cmdVelTwist.linear.x = 0.0;
//    cmdVelTwist.linear.y = 0.0;
//    cmdVelTwist.linear.z = 0.0;
//    cmdVelTwist.angular.x = 0.0;
//    cmdVelTwist.angular.y = 0.0;
//    cmdVelTwist.angular.z = 0.0;

//    geometry_msgs::Point targetPosBaseLink;
//    getPoint(targetPosMap, "/base_link", "/map", targetPosBaseLink);

//    float targetAngle = std::atan2(targetPosBaseLink.y , targetPosBaseLink.x);
//    ROS_INFO("adjustYaw targetAngle: %f", targetAngle);

//    int loopIters = 0;
//    while (std::fabs(targetAngle) > angular_threshold) {
//      ROS_DEBUG("iteration %d ang correct", loopIters);
//      loopIters++;

//      if (targetAngle > 0.5){
//        targetAngle = 0.5;
//      } else if (targetAngle < -0.5){
//        targetAngle = -0.5;
//      }

//      //Turtlebot won't turn for rates below 0.1
//      if ((targetAngle < 0.1) && (targetAngle >= 0)){
//        targetAngle = 0.1;
//      }else if ((targetAngle < -0.1) && (targetAngle < 0)){
//        targetAngle = -0.1;
//      }

//      ROS_DEBUG("adjustYaw used targetAngle: %f", targetAngle);

//      cmdVelTwist.angular.z = targetAngle;

//      //ROS_INFO("%s", cmdVelTwist);

//      cmdVelPublisher.publish(cmdVelTwist);
//      getPoint(targetPosMap, "/base_link", "/map", targetPosBaseLink);

//      targetAngle = std::atan2(targetPosBaseLink.y , targetPosBaseLink.x);
//      ROS_DEBUG("adjustYaw targetAngle: %f", targetAngle);
//      ros::Duration(0.1).sleep();

//      if (_state == STATE_FIND_TARGET_BALL)
//        return false;
//    }
//    cmdVelTwist.angular.z = targetAngle;
//    cmdVelPublisher.publish(cmdVelTwist);

//    return true;
//  }


//  bool adjustTranslation(const geometry_msgs::Point& targetPosMap, double dist_threshold)
//  {
//    // initialize Twist-Message
//    geometry_msgs::Twist cmdVelTwist;
//    cmdVelTwist.linear.x = 0.0;
//    cmdVelTwist.linear.y = 0.0;
//    cmdVelTwist.linear.z = 0.0;
//    cmdVelTwist.angular.x = 0.0;
//    cmdVelTwist.angular.y = 0.0;
//    cmdVelTwist.angular.z = 0.0;

//    geometry_msgs::Point targetPosBaseLink;

//    getPoint(targetPosMap, "/base_link", "/map", targetPosBaseLink);

//    //      float targetAngle = std::atan2(targetPosBaseLink.y , targetPosBaseLink.x);
//    //      approachBallGetCurVelData(&angle, &dist);
//    float dist = getEuclidDist(targetPosBaseLink.x, targetPosBaseLink.y);
//    ROS_DEBUG("adjustTranslation dist: %f", dist);

//    int loopIters = 0;
//    while (dist > dist_threshold){
//      //        if (dist > 0.5)
//      //          dist = 0.5;
//      ROS_DEBUG("iteration %d trans correct", loopIters);

//      dist = 0.2;
//      cmdVelTwist.angular.z = 0.0;
//      cmdVelTwist.linear.x = dist;

//      //Turtlebot can't go slower that this
//      if (cmdVelTwist.linear.x < 0.021){
//        cmdVelTwist.linear.x = 0.021;
//      }



//      cmdVelPublisher.publish(cmdVelTwist);
//      getPoint(targetPosMap, "/base_link", "/map", targetPosBaseLink);
//      dist = getEuclidDist(targetPosBaseLink.x, targetPosBaseLink.y ) ;
//      ROS_DEBUG("adjustTranslation new dist: %f", dist);
//      ros::Duration(0.1).sleep();
//    }
//    cmdVelTwist.angular.z = 0.0;
//    cmdVelTwist.linear.x = 0.0;
//    cmdVelPublisher.publish(cmdVelTwist);
//    return true;
//  }

//  void driveToGoal()
//  {
//    while (_state == STATE_DRIVE_TO_GOAL) {
//      // UNTESTED
//      move_base_msgs::MoveBaseGoal goal;

//      // we'll send a goal to the robot
//      goal.target_pose.header.frame_id = "map";
//      goal.target_pose.header.stamp = ros::Time::now();

//      goal.target_pose.pose.position.x = 0.5;
//      goal.target_pose.pose.position.y = 0.0;
//      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(3.14);

//      ROS_INFO("Sending goal");
//      mbClient->sendGoal(goal);

//      mbClient->waitForResult();

//      if (mbClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
//        ROS_INFO("Great we reached the goal");


//        geometry_msgs::Point final_goal_pos;
//        final_goal_pos.x = 0.2;
//        final_goal_pos.y = 0.0;
//        final_goal_pos.z = 0.0;

//        this->adjustYaw(final_goal_pos, 0.05);
//        this->adjustTranslation(final_goal_pos, 0.4);

//        this->adjustYaw(final_goal_pos, 0.025);
//        this->adjustTranslation(final_goal_pos, 0.2);

//        setState(STATE_DROP_BALL);
//        return;
//      } else {
//        ROS_INFO("That was bad, we didn't reach our goal");
//      }
//    }
//  }



//  void recedeFromGoal()
//  {
//    // set manipulator position
//    manipulator_drive();

//    // TODO

//    geometry_msgs::Twist cmdVelTwist;
//    cmdVelTwist.linear.x = -0.5;
//    cmdVelTwist.linear.y = 0.0;
//    cmdVelTwist.linear.z = 0.0;
//    cmdVelTwist.angular.x = 0.0;
//    cmdVelTwist.angular.y = 0.0;
//    cmdVelTwist.angular.z = 0.0;

//    // UNCLEAN but not sure how to publish more then one message for a specified time
//    geometry_msgs::Point pos;
//    geometry_msgs::Point posGoal;
//    posGoal.x = 0.0;
//    posGoal.y = 0.0;
//    posGoal.z = 0.0;
//    float yaw;
//    do {
//      cmdVelPublisher.publish(cmdVelTwist);
//      ros::Duration(1.0).sleep();
//      getCurrentPosition(&pos, &yaw);
//    } while (distance(&pos, &posGoal) <= 1.0);

//    // turn around
//    getCurrentPosition(&pos, &yaw);

//    move_base_msgs::MoveBaseGoal goal;

//    // we'll send a goal to the robot
//    goal.target_pose.header.frame_id = "map";
//    goal.target_pose.header.stamp = ros::Time::now();

//    goal.target_pose.pose.position.x = pos.x;
//    goal.target_pose.pose.position.y = pos.y;
//    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

//    ROS_INFO("Sending goal");
//    mbClient->sendGoal(goal);

//    mbClient->waitForResult();

//    setState(STATE_FIND_TARGET_BALL);
//  }



  void getCurrentPosition(geometry_msgs::Point *pos, float *yaw)
  {
    tf::StampedTransform trans;
    try {
      ros::Time now = ros::Time::now();
      tf_listener.waitForTransform("/map", "/base_link", now, ros::Duration(4.0));
      // get current position and rotation in quanternions
      tf_listener.lookupTransform("/map", "/base_link", now, trans);
    } catch (tf::TransformException ex) {
      ROS_INFO("getCurrentPosition_TF_Error: %s",ex.what());
      return;
    }


    // get position
    pos->x = trans.getOrigin().x();
    pos->y = trans.getOrigin().y();
    pos->z = trans.getOrigin().z();

    // get yaw-angle
    *yaw = tf::getYaw(trans.getRotation());

    ROS_DEBUG("getCurrentPosition:");
    ROS_DEBUG("  pos = [%f,%f,%f]", trans.getOrigin().x(), trans.getOrigin().y(), trans.getOrigin().z());
    ROS_DEBUG("  yaw = %f", tf::getYaw(trans.getRotation()));
  }

  bool getPose(const geometry_msgs::Pose& pose, const std::string& target_frame, const std::string& source_frame, geometry_msgs::Pose& pose_out){
    //tf::StampedTransform trans;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose = pose;
    pose_stamped.header.frame_id = source_frame;
    pose_stamped.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped pose_stamped_out;

    try {
      ros::Time time = ros::Time::now();
      tf_listener.waitForTransform(target_frame, source_frame, time, ros::Duration(0.5));
      // get current position and rotation in quanternions
      //tf_listener.lookupTransform(target_frame, source_frame, time, trans);
      tf_listener.transformPose(target_frame, pose_stamped, pose_stamped_out );
    } catch (tf::TransformException ex) {
      ROS_INFO("getPose tf error: %s",ex.what());
      return false;
    }

    pose_out = pose_stamped_out.pose;
    return true;
  }


  bool getPoint(const geometry_msgs::Point& point, const std::string& target_frame, const std::string& source_frame, geometry_msgs::Point& point_out)
  {
    geometry_msgs::Pose pose_in;
    pose_in.position = point;
    pose_in.orientation.w = 1.0;

    geometry_msgs::Pose pose_out;

    if (getPose(pose_in, target_frame, source_frame, pose_out)){
      point_out = pose_out.position;
      return true;
    } else{
      return false;
    }
  }

  /*
  void getPose(const geometry_msgs::Pose& pose, const std::string& source_frame, const std::string& target_frame, geometry_msgs::Pose& pose_out){


  }
  */


private:
  ros::NodeHandle nh_;

  ros::Publisher cmdVelPublisher;
  ros::Publisher messagePublisher;

  ros::Subscriber modelSubscriber;
  ros::Subscriber ballInGrabberSubscriber;

  tf::TransformListener tf_listener;

  // class-id of the target balls
  std::string          _ball_color;

  // object-id and position of a specific target ball
  std::string          _target_id;
  geometry_msgs::Point _target_pos;

  // object-id and position of the nearest ball
  std::string          _nearest_id;
  geometry_msgs::Point _nearest_pos;


  bool _run;

  enum State {
      STATE_IDLE,
    STATE_START,
    STATE_STOP,
    STATE_DRIVE_1m
  };

  std::string getStateName(State _state)
  {
    switch (_state) {
    case STATE_IDLE:
      return "STATE_IDLE";
      case STATE_START:
        return "STATE_START";
      case STATE_STOP:
        return "STATE_STOP";
      case STATE_DRIVE_1m:
        return "STATE_DRIVE_1m";

      default:
        return "undefined state";
    };
  }

  State _state;
  bool  _state_lock;

  State _nextSingleState;
  bool _executeSingleState;

  void setState(State newState) {
    if (!_state_lock)
      _state = newState;
  }

  boost::thread m_Thread0, m_Thread1;



  MoveBaseClient *mbClient;




  void objectsCb(const hector_worldmodel_msgs::ObjectModelConstPtr& objectModel)
  {/*
    int size = objectModel->get_objects_size();
    ROS_DEBUG("in objects cb. Size: %d", size);

    std::string          new_id;
    float                new_dist = FLT_MAX;
    geometry_msgs::Point new_pos;

    geometry_msgs::Point target_update_pos;

    geometry_msgs::Point my_pos;
    float                my_yaw;
    getCurrentPosition(&my_pos, &my_yaw);

    bool hasTarget = false;

    // search for objects with classId == _ball_color
    for(int i=0; i<size; i++) {
      hector_worldmodel_msgs::Object object = objectModel->objects[i];

      // check for nearest ball
      if (std::strcmp(object.info.class_id.c_str(), _ball_color.c_str()) == 0) {
        // check distance
        float dist = distance(&my_pos, &object.pose.pose.position);
        if (dist < new_dist) {
          // this object is closer
          new_id   = object.info.object_id;
          new_dist = dist;
          new_pos  = object.pose.pose.position;
        }
      }

      // check for _target_id
      if (hasTargetBall() && std::strcmp(object.info.object_id.c_str(), _target_id.c_str()) == 0) {
        hasTarget         = true;
        target_update_pos = object.pose.pose.position;
      }
    }

    std_msgs::String msg;
    if (std::strcmp(_nearest_id.c_str(), new_id.c_str()) != 0) {
      msg.data = "setting nearest id to " + new_id;
      messagePublisher.publish(msg);
    }

    // object found
    if (std::strlen(new_id.c_str()) > 0) {
      _nearest_id  = new_id;
      new_pos.z = 0;
      _nearest_pos = new_pos;
    } else {
      _nearest_id  = "";
    }

    // target found => update position
    if (hasTarget)
      _target_pos = target_update_pos;
    else
      _target_id = "";*/
  }




  ros::ServiceServer startMission;

  bool startMissionCb(sickrobot_2014_msgs::StartMission::Request& request, sickrobot_2014_msgs::StartMission::Response& response) {
    if (!request.start) {
      _state_lock = true;
      _state = STATE_START;
      // send abort signal to MoveBase
     // mbClient->cancelAllGoals();
    }
    if(request.start){
     _state = STATE_START;}

    _run = request.start;

    ROS_INFO("start = %i", _run);

    return true;
  }

  ros::ServiceServer executeState;

  bool executeStateCb(sickrobot_2014_msgs::ExecuteState::Request& request, sickrobot_2014_msgs::ExecuteState::Response& response) {
    _run = false;
    _state_lock = true;
    _state = STATE_START; // this was idle before need to check this :)
    _executeSingleState = true;
    _nextSingleState = (State)request.state;

    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);
  sickrobot_behaviour behaviour;

  // start state machine thread
  behaviour.start0();

  // start checkTarget thread
  behaviour.start1();

  ROS_INFO("spin started");


  ros::spin();

  return 0;
}
