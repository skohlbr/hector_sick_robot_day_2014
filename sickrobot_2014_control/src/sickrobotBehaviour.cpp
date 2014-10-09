#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <hector_worldmodel_msgs/ObjectModel.h>
#include <hector_worldmodel_msgs/Object.h>
#include <hector_worldmodel_msgs/AddObject.h>
#include <hector_worldmodel_msgs/GetObjectModel.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
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
#include <visualization_msgs/MarkerArray.h>
#include <vector>

#include <hector_nav_msgs/GetDistanceToObstacle.h>

#include <sstream>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class sickrobot_behaviour {
public:

    sickrobot_behaviour()
    {
         struct Vector2D{
            float x;
            float y;
        };
        // instantiate publisher and subscriber
        ros::NodeHandle worldmodel(std::string("worldmodel"));
        worldmodel_AddObject = worldmodel.serviceClient<hector_worldmodel_msgs::AddObject>("add_object");
        worldmodel.param("exploration_dist_wall", explor_dist_wall, 1.0);
        worldmodel.param("exploration_circle_radius", exploration_circle_radius, 6.0);
        ros::NodeHandle root("");
       cmdVelPublisher = root.advertise<geometry_msgs::Twist>("turtlebot_node/cmd_vel", 10, false);
        messagePublisher = root.advertise<std_msgs::String>("sickrobot/print", 10, false);

        holzklotzSubscriber = root.subscribe<std_msgs::Int32>("holzklotz_number", 10, &sickrobot_behaviour::holzklotzCb, this);
        //worldmodel_objects_sub = root.subscribe<std_msgs::Int32>("holzklotz_number", 10, &sickrobot_behaviour::holzklotzCb, this);
        modelSubscriber = worldmodel.subscribe("objects", 10, &sickrobot_behaviour::objectsCb, this);


        ros::NodeHandle sick_nh(std::string("sickrobot"));
        startMission = sick_nh.advertiseService("start_mission", &sickrobot_behaviour::startMissionCb, this);
        m_marker_points = sick_nh.advertise<visualization_msgs::MarkerArray>("endpoints", 1, false);
        m_marker_normal = sick_nh.advertise<visualization_msgs::MarkerArray>("normal", 1, false);

        getDist_client = root.serviceClient<hector_nav_msgs::GetDistanceToObstacle>("hector_map_server/get_distance_to_obstacle");

        //    executeState = sick_nh.advertiseService("execute_state", &sickrobot_behaviour::executeStateCb, this);


        _run = false;
        _has_cargo=false;
        _executeSingleState = false;
        _nextSingleState = STATE_IDLE;
        present_holzklotz=-1;


        // get moveBaseClient
        // tell the action client that we want to spin a thread by default
        mbClient = new MoveBaseClient ("move_base", true);
        // wait for the action server to come up
        while (!mbClient->waitForServer(ros::Duration(5.0)))
            ROS_INFO("Waiting for the move_base action server to come up");

    }


    ~sickrobot_behaviour() {
        _state = STATE_STOP;

        delete mbClient;
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

        //add_stations_to_worldmodel(0,0,0,"ladestation", "0", "Ladestation 0");
        //add_stations_to_worldmodel(-3.5,0,0,"entladestation", "1", "Entladestation 1");

        while (true) {
            if (_state == STATE_IDLE ||
                    // _state == STATE_DRIVE_TO_GOAL ||
                    //_state == STATE_ERROR ||
                    _state == STATE_STOP
                    ) {
                // do nothing
                ROS_INFO("do nothing");
            } //else if (_state == STATE_DRIVE_1m){
            else{
              //  ROS_INFO("do something");
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
            // msg.data = getStateName(_state);
            //   messagePublisher.publish(msg);
            //  ROS_INFO("_state = %i (%s)", _state, getStateName(_state).c_str());

            switch(_state) {
            case STATE_START:
                start_task();
                break;
            case STATE_DRIVE_TO_FIRST_LOAD_STATION:
                drive_to_frist_load_station_task();
                break;
            case STATE_POSITIONING:
                positioning_task();
            case STATE_LOAD_CARGO:
                load_cargo_task();
                break;
            case STATE_FIND_UNLOAD_STATION:
                find_unload_station_task();
                break;
            case STATE_DRIVE_TO_UNLOAD_STATION:
                drive_to_unload_station_task();
                break;
            case STATE_EXPLORATION:
                exploration_task();
            case STATE_UNLOAD_CARGO:
                unload_cargo_task();
            case STATE_DRIVE_TO_LOAD_STATION:
                drive_to_load_station_task();
            case STATE_STOP:
                _state_lock = false;
                break;

//            case STATE_DRIVE_1m:
//                // driveToGoal();
//                drive_1m();
//                break;
            };

            //      if (_executeSingleState) {
            //        _executeSingleState = false;
            //        _state_lock = false;
            //        _state = STATE_START;
            //      }
        }

        ROS_INFO("sm exited with state %i", _state);
    }


    hector_worldmodel_msgs::AddObject srv_call;

    //    void add_stations_to_worldmodel(double position_x,double position_y,double position_z, std::string class_id, std::string object_id, std::string name){

    //        srv_call.request.object.header.stamp = ros::Time::now();
    //        srv_call.request.object.header.frame_id = "map";
    //        srv_call.request.object.header.seq++;

    //        srv_call.request.object.pose.pose.position.x = position_x;
    //        srv_call.request.object.pose.pose.position.y = position_y;
    //        srv_call.request.object.pose.pose.position.z = position_z;

    //        srv_call.request.object.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    //        srv_call.request.object.info.class_id = class_id;
    //        srv_call.request.object.info.object_id = object_id;
    //        srv_call.request.object.info.name = name;
    //        srv_call.request.object.info.support = 1.0;
    //        srv_call.request.object.state.state = 70;

    //        if(!worldmodel_AddObject.call(srv_call)){
    //            ROS_ERROR("Failed to add object to worldmodel");
    //        }

    //    }

    void start_task()
    {
        //add_stations_to_worldmodel(0,0,0,"ladestation", "0", "Ladestation 0"); //Insert current position as start_position
        ROS_INFO("state: start");

        _state = STATE_DRIVE_TO_FIRST_LOAD_STATION;
    }



    void drive_to_frist_load_station_task(){
        //drive straight until first goal station is seen, then drive towards it till certain distance threshold
        //TODO make an adaptive step control ( if not possible to plan through wall etc)
        ROS_INFO("state: driving to first load station");
                while (_number_objects==0){
                geometry_msgs::Point my_pos;
                float                my_yaw;

                getCurrentPosition(&my_pos, &my_yaw);

                std::cout << "pos x:" <<my_pos.x << "   pos_y:"<<my_pos.y << "   my pos z:" <<my_pos.z<<   "yaw  " <<my_yaw <<std::endl;
                move_base_msgs::MoveBaseGoal goal;
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();


                //TODO need to ckeck if we really need yaw cos sin here !!!! maybe drifting because of this
                goal.target_pose.pose.position.x = my_pos.x+0.75;//*cos(my_yaw);
                goal.target_pose.pose.position.y = my_pos.y;//+1*sin(my_yaw);
                goal.target_pose.pose.position.z = my_pos.z;
                goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

                //_state = STATE_DRIVE_TO_GOAL;
               // driveToGoal(goal);

                ROS_INFO("Sending goal");
                           mbClient->sendGoal(goal);

                           mbClient->waitForResult();

                           if (mbClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                               ROS_INFO("Great we reached the goal");

                              // setState(STATE_STOP);
                              // return;
                           }

                           else {
                               ROS_INFO("That was bad, we didn't reach our goal");
                           }

              }
                current_target = objects->objects[0];
                first_load_station=objects->objects[0];
        _state = STATE_POSITIONING;
    }

    void positioning_task(){
        // try to reach a good position under the ring
        ROS_INFO("state: positioning");
        geometry_msgs::PointStamped point;
        point.point=current_target.pose.pose.position;
        point.header.frame_id=current_target.header.frame_id;
        point.header.stamp=current_target.header.stamp;
        float normal_slope_x;
        float normal_slope_y;
        getNormal(point,normal_slope_x,normal_slope_y);

        geometry_msgs::Point my_pos;
        float                my_yaw;

        getCurrentPosition(&my_pos, &my_yaw);
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        float dist_wall=0.6;
        goal.target_pose.pose.position.x = current_target.pose.pose.position.x+dist_wall*normal_slope_x;
        goal.target_pose.pose.position.y = current_target.pose.pose.position.y+dist_wall*normal_slope_y;
        goal.target_pose.pose.position.z = my_pos.z;
        std::cout<< " yaw for position " <<(atan2(normal_slope_y,normal_slope_x)/M_PI)*180<<std::endl;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI+atan2(normal_slope_y,normal_slope_x));

        ROS_INFO("Sending goal");
                   mbClient->sendGoal(goal);

                   mbClient->waitForResult();

                   if (mbClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                       ROS_INFO("Great we reached the goal");
                        //initialize Twist-Message
                               geometry_msgs::Twist cmdVelTwist;
                               cmdVelTwist.linear.x = 0.0;
                               cmdVelTwist.linear.y = 0.0;
                               cmdVelTwist.linear.z = 0.0;
                               cmdVelTwist.angular.x = 0.0;
                               cmdVelTwist.angular.y = 0.0;
                               cmdVelTwist.angular.z = 0.0;

                               getCurrentPosition(&my_pos, &my_yaw);
                               float offset_error;
                               offset_error=(std::abs(my_pos.x- current_target.pose.pose.position.x)-dist_wall);
                               std::cout << "ofset error " <<offset_error<<std::endl;

                               std::cout << "my pos " << my_pos.x<<  "   target " <<current_target.pose.pose.position.x <<" norm  "<<0.25*normal_slope_x<<std::endl;
                               //std::cout << " dist to target " << abs((my_pos.x- current_target.pose.pose.position.x)+(0.2*normal_slope_x)) << std::endl;
                               std::cout << " dist to target " << std::abs(my_pos.x- current_target.pose.pose.position.x)+0.25*normal_slope_x-offset_error << std::endl;
                                //std::cout << " dist to target " << ((my_pos.x- current_target.pose.pose.position.x)+(0.2*normal_slope_x)) << std::endl;
                               //                              getPoint(targetPosMap, "/base_link", "/map", targetPosBaseLink);

//                               //      float targetAngle = std::atan2(targetPosBaseLink.y , targetPosBaseLink.x);
//                               //      approachBallGetCurVelData(&angle, &dist);
//                               float dist = getEuclidDist(targetPosBaseLink.x, targetPosBaseLink.y);
//                               ROS_DEBUG("adjustTranslation dist: %f", dist);

//                               int loopIters = 0;
                               while (((std::abs(my_pos.x- current_target.pose.pose.position.x)+0.25*normal_slope_x-offset_error)>0.05)){
                                 //        if (dist > 0.5)
                                 //          dist = 0.5;

                                   std::cout << " dist to target " << std::abs(my_pos.x- current_target.pose.pose.position.x)+0.25*normal_slope_x-offset_error << std::endl;

                                 cmdVelTwist.linear.x = 0.1;





                                 cmdVelPublisher.publish(cmdVelTwist);

                                 ros::Duration(0.1).sleep();
                                 getCurrentPosition(&my_pos, &my_yaw);
                               }

                               cmdVelTwist.linear.x = 0.0;              
                               cmdVelTwist.angular.z = 0.2;
                               cmdVelPublisher.publish(cmdVelTwist);

                               while (((std::abs(my_yaw-atan2(normal_slope_y,normal_slope_x)))>0.05)){


                                   cmdVelPublisher.publish(cmdVelTwist);
                                    ros::Duration(0.1).sleep();
                                    getCurrentPosition(&my_pos, &my_yaw);

                               }
                               cmdVelTwist.linear.x = 0.0;
                               cmdVelTwist.angular.z = 0.0;
                               cmdVelPublisher.publish(cmdVelTwist);





                       if (!_has_cargo){

                       _state=STATE_LOAD_CARGO;}
                       else {
                           _state=STATE_UNLOAD_CARGO;
                       }
                      // return;
                   }

                   else {
                       ROS_INFO("That was bad, we didn't reach our goal");
                   }





    }

    void load_cargo_task(){
        ROS_INFO("state: load_cargo");
        while (!_has_cargo){
        if(present_holzklotz >= 0){
            _has_cargo=true;
            _state = STATE_FIND_UNLOAD_STATION;
            break;
            //set the number of current unload station
        }
        ros::Duration(0.1).sleep();
        }

    }


    void find_unload_station_task(){
        ROS_INFO("state: find_unload_station");
        bool station_known=false;
        //TODO find unload station in worldmodel
        for (int i=0;i< objects->objects.size();i++){
            hector_worldmodel_msgs::Object object = objects->objects[i];
            std::stringstream sstr;
            sstr << object.info.name;
            int curr_number_unload_station;
            sstr >> curr_number_unload_station;
            if ((object.info.class_id=="unload_station")&&(present_holzklotz==curr_number_unload_station)){
                station_known=true;
            }

        }
        if (station_known){
        _state = STATE_DRIVE_TO_UNLOAD_STATION;}
        else{
            _state=STATE_EXPLORATION;
        }
    }

    void find_load_station(){
        ROS_INFO("state: find_load_station");
        bool found_one=false;
        //TODO find unload station in worldmodel

        if (found_one){
        _state = STATE_DRIVE_TO_LOAD_STATION ;}
        else{
            _state=STATE_EXPLORATION;
        }
    }

    void exploration_task(){
        //do this kind of wall following till current target station is known
        bool is_known=false;
        geometry_msgs::Point mp;
        mp.x=exploration_circle_radius;
        mp.y=0;
        mp.z=0;



        int phi=0;
        while (!is_known){
            if (phi > 360){
                phi=0;
            }




            geometry_msgs::Point my_pos;
            float                my_yaw;
            getCurrentPosition(&my_pos, &my_yaw);

            move_base_msgs::MoveBaseGoal circle_point;
            circle_point.target_pose.pose.position.x = (exploration_circle_radius-explor_dist_wall)*cos((phi/180)*M_PI)+mp.x; //TODO use pose from worldmodel
            circle_point.target_pose.pose.position.y =(exploration_circle_radius-explor_dist_wall)*sin((phi/180)*M_PI)+mp.y;
            circle_point.target_pose.pose.position.z = my_pos.z;

            circle_point.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(((3/2)*M_PI)+phi);

            while (_state == STATE_DRIVE_TO_UNLOAD_STATION) {

                ROS_INFO("Sending goal");
                mbClient->sendGoal(circle_point);

                mbClient->waitForResult();

                if (mbClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("Great we reached the goal");

                   // setState(STATE_STOP);
                    _state=STATE_POSITIONING;
                    return;
                } else {
                    ROS_INFO("That was bad, we didn't reach our goal");
                }
            }
         phi++;
        }
        _state=STATE_DRIVE_TO_UNLOAD_STATION;
    }

    void drive_to_unload_station_task(){

        //think about station could be blocked here !!!! Add some wait state
        ROS_INFO("state: drive_to_unload_station");
        move_base_msgs::MoveBaseGoal station;

        station.target_pose.header.frame_id = "map";
        station.target_pose.header.stamp = ros::Time::now();


        geometry_msgs::Point my_pos;
        float                my_yaw;

        getCurrentPosition(&my_pos, &my_yaw);
        station.target_pose.pose.position.x = my_pos.x +0.50; //TODO use pose from worldmodel
        station.target_pose.pose.position.y = my_pos.y;
        station.target_pose.pose.position.z = my_pos.z;

        station.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(my_yaw);

        while (_state == STATE_DRIVE_TO_UNLOAD_STATION) {

            ROS_INFO("Sending goal");
            mbClient->sendGoal(station);

            mbClient->waitForResult();

            if (mbClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Great we reached the goal");

               // setState(STATE_STOP);
                _state=STATE_POSITIONING;
                return;
            } else {
                ROS_INFO("That was bad, we didn't reach our goal");
            }
        }
    }

    void unload_cargo_task(){

        ROS_INFO("state: unload_cargo");
        while (_has_cargo){
        if(present_holzklotz < 0){ // check if scanner sending -1 in case of no detection
            _has_cargo=true;
            _state = STATE_DRIVE_TO_LOAD_STATION;
            break;
            //set the number of current unload station
        }
        ros::Duration(0.1).sleep();
        }

    }

    void drive_to_load_station_task(){
     //look for next loading station, maybe choose always "our" one as other teams maybe also do this

        _state=STATE_POSITIONING;
    }

    void drive_1m()
    {
//        ROS_INFO("state: drive_1m");

//        geometry_msgs::Point my_pos;
//        float                my_yaw;

//        getCurrentPosition(&my_pos, &my_yaw);

//        std::cout << "pos x:" <<my_pos.x << "   pos_y:"<<my_pos.y << "   my pos z:" <<my_pos.z<<std::endl;
//        move_base_msgs::MoveBaseGoal goal;
//        goal.target_pose.header.frame_id = "map";
//        goal.target_pose.header.stamp = ros::Time::now();

//        goal.target_pose.pose.position.x = my_pos.x+0.5;
//        goal.target_pose.pose.position.y = my_pos.y;
//        goal.target_pose.pose.position.z = my_pos.z;
//        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(my_yaw);

//        _state = STATE_DRIVE_TO_GOAL;
//        driveToGoal(goal);
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

//    void driveToGoal(move_base_msgs::MoveBaseGoal goal)
//    {
//        while (_state == STATE_DRIVE_TO_GOAL) {
//            // UNTESTED
//            //move_base_msgs::MoveBaseGoal goal;

//            // we'll send a goal to the robot
//            //      goal.target_pose.header.frame_id = "map";
//            //      goal.target_pose.header.stamp = ros::Time::now();

//            //      goal.target_pose.pose.position.x = 0.5;
//            //      goal.target_pose.pose.position.y = 0.0;
//            //      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(3.14);

//            ROS_INFO("Sending goal");
//            mbClient->sendGoal(goal);

//            mbClient->waitForResult();

//            if (mbClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
//                ROS_INFO("Great we reached the goal");


//                //        geometry_msgs::Point final_goal_pos;
//                //        final_goal_pos.x = 0.2;
//                //        final_goal_pos.y = 0.0;
//                //        final_goal_pos.z = 0.0;

//                //        this->adjustYaw(final_goal_pos, 0.05);
//                //        this->adjustTranslation(final_goal_pos, 0.4);

//                //        this->adjustYaw(final_goal_pos, 0.025);
//                //        this->adjustTranslation(final_goal_pos, 0.2);

//                setState(STATE_STOP);
//                return;
//            } else {
//                ROS_INFO("That was bad, we didn't reach our goal");
//            }
//        }
//    }



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

    void getNormal(geometry_msgs::PointStamped point,float &normal_slope_x,float &normal_slope_y){
        geometry_msgs::Point my_pos;
        float                my_yaw;

        getCurrentPosition(&my_pos, &my_yaw);

        hector_nav_msgs::GetDistanceToObstacle getDist_srv;


        std::vector<float> distances;
        std::vector<geometry_msgs::Point> end_points;
        std::vector<geometry_msgs::Point> directions;
        geometry_msgs::PointStamped direction=point;
        tf::Point pointTF;
        tf::Point directionTF;
        geometry_msgs::Point end_point;

        for (int i=0;i<5;i++){
        getDist_srv.request.point=direction;
        getDist_srv.request.point.header.stamp = ros::Time::now();
        getDist_srv.request.point.header.frame_id="base_link";

        getDist_client.call(getDist_srv);
        float dis=getDist_srv.response.distance;
        distances.push_back(dis);
        if (i==0){
        direction.point.x=direction.point.x+0.5;}
        else if (i==1){
        direction.point.x=direction.point.x-1.0;
        }
        else if (i==2){
        direction.point.x=direction.point.x+0.5;
        direction.point.y=direction.point.y+0.5;
        }
        else if (i==3){
        direction.point.y=direction.point.y-1.0;
        }


       float diffx= direction.point.x-my_pos.x;
       float diffy= direction.point.y-my_pos.y;

       float norm_factor=sqrt(diffx*diffx+diffy*diffy);

       float norm_x=diffx*(1/norm_factor);
       float norm_y=diffy*(1/norm_factor);

       end_point.x=my_pos.x+dis*norm_x;
       end_point.y=my_pos.y+dis*norm_y;
       end_point.z=point.point.z;

        end_points.push_back(end_point);
        directions.push_back(direction.point);


        }

        if (m_marker_points.getNumSubscribers() > 0  ){

            visualization_msgs::MarkerArray marker_array;

            visualization_msgs::Marker marker;
            marker.header.stamp = point.header.stamp;
            marker.header.frame_id = point.header.frame_id;
            marker.type = visualization_msgs::Marker::POINTS;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.r= 1.0;
            marker.color.a = 1.0;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.ns ="";
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;

            std::vector<geometry_msgs::Point> point_vector;

            for (size_t i = 0; i < end_points.size(); ++i){

                point_vector.push_back(end_points[i]);
            }

            marker.points=point_vector;
            marker_array.markers.push_back(marker);
            m_marker_points.publish(marker_array);

        }




        std::vector<geometry_msgs::Point> regress_points;
        for (int i=0;i<end_points.size();i++){
            if(sqrt((point.point.x-end_points[i].x)*(point.point.x-end_points[i].x)+(point.point.y-end_points[i].y)*(point.point.y-end_points[i].y))<1){
                regress_points.push_back(end_points[i]);
        }
        }

        std::vector<geometry_msgs::Point> regress_points_no_duplicate_x;
        for (int i=0;i<regress_points.size();i++){
            bool duplicate=false;
            for (int j=i+1;j<regress_points.size();j++){
                if (abs(regress_points[i].x-regress_points[j].x)<0.1){
                    duplicate=true;
                    break;
                }
            }
            if (!duplicate){
                regress_points_no_duplicate_x.push_back(regress_points[i]);
            }

        }


        float b;
        if (regress_points_no_duplicate_x.size()>1){
        // calculate the slope of the wall line
        float sumx=0;
        float sumy=0;
        float sumxy=0;
        float sumxx=0;
        for (int i=0;i<regress_points_no_duplicate_x.size();i++){
            sumx=sumx+regress_points_no_duplicate_x[i].x;
            sumy=sumy+regress_points_no_duplicate_x[i].y;
        }
        float meanx=sumx/regress_points_no_duplicate_x.size();
        float meany=sumy/regress_points_no_duplicate_x.size();

        for (int i=0;i<regress_points_no_duplicate_x.size();i++){
            sumxy=sumxy+(regress_points_no_duplicate_x[i].x-meanx)*(regress_points_no_duplicate_x[i].y-meany);
            sumxx=sumxx+(regress_points_no_duplicate_x[i].x-meanx)*(regress_points_no_duplicate_x[i].x-meany);
        }

        float m=sumxy/sumxx;
        b=meany-m*meanx;

        // line is parallel to x-axis, need to treat this special
        if (abs(m)<0.01){
            b=meany-m*meanx;
            normal_slope_x=0;

            if (my_pos.y>point.point.y){
            normal_slope_y=1;}
            else{
               normal_slope_y=-1;
            }
        }

         else{
        normal_slope_x=1;
         normal_slope_y=-1/m; }
         std::cout <<"m:" <<std::endl;
         ROS_INFO_STREAM(m);

        }

        // line is parallel to y-axis need to treat this special
        else{
            if (my_pos.x>point.point.x){
            normal_slope_x=1;}
            else{
                normal_slope_x=-1;
            }
            normal_slope_y=0;
            b= regress_points_no_duplicate_x[0].y;

        }
std::cout <<"normal_slope_x:" <<std::endl;
        ROS_INFO_STREAM(normal_slope_x);
        std::cout <<"normal_slope_y:" <<std::endl;
        ROS_INFO_STREAM(normal_slope_y);
        std::cout <<"b:" <<std::endl;
        ROS_INFO_STREAM(b);
        if (m_marker_normal.getNumSubscribers() > 0  ){


                visualization_msgs::MarkerArray marker_array;
                visualization_msgs::Marker marker;
                marker.header.stamp = point.header.stamp;
                marker.header.frame_id = point.header.frame_id;
                marker.type = visualization_msgs::Marker::LINE_LIST;
                marker.action = visualization_msgs::Marker::ADD;
                marker.color.r= 1.0;
                marker.color.a = 1.0;
                marker.scale.x = 0.02;
                marker.ns ="";
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.orientation.w = 1.0;

                std::vector<geometry_msgs::Point> point_vector;
                geometry_msgs::Point p1;

                    point_vector.push_back(point.point);
                    p1.x=point.point.x+1*normal_slope_x;
                    p1.y=point.point.y+1*normal_slope_y;
                    p1.z=point.point.z;
                    point_vector.push_back(p1);

                marker.points=point_vector;
                marker_array.markers.push_back(marker);
                m_marker_normal.publish(marker_array);
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

     ros::Publisher m_marker_points;
     ros::Publisher m_marker_normal;

    ros::Subscriber modelSubscriber;
    ros::Subscriber ballInGrabberSubscriber;

    ros::Subscriber holzklotzSubscriber;
    ros::Subscriber worldmodel_objects_sub;

    tf::TransformListener tf_listener;


    ros::ServiceClient getDist_client;
    // class-id of the target balls
    std::string          _ball_color;

    // object-id and position of a specific target ball
    std::string          _target_id;
    geometry_msgs::Point _target_pos;

    // object-id and position of the nearest ball
    std::string          _nearest_id;
    geometry_msgs::Point _nearest_pos;


    bool _run;
    bool _has_cargo;
    int _number_objects;
    hector_worldmodel_msgs::ObjectModelConstPtr objects;
   // std::string current_drive_target;
    hector_worldmodel_msgs::Object current_target;
    hector_worldmodel_msgs::Object first_load_station;
    double explor_dist_wall;
    double exploration_circle_radius;

    enum State {
        STATE_IDLE,
        STATE_START,
        STATE_STOP,
        STATE_DRIVE_TO_LOAD_STATION,
        STATE_LOAD_CARGO,
        STATE_FIND_UNLOAD_STATION,
        STATE_DRIVE_TO_UNLOAD_STATION,
        STATE_DRIVE_TO_FIRST_LOAD_STATION,
        STATE_POSITIONING,
        STATE_EXPLORATION,
        STATE_UNLOAD_CARGO
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
//        case STATE_DRIVE_1m:
//            return "STATE_DRIVE_1m";
        case STATE_DRIVE_TO_FIRST_LOAD_STATION:
            return "STATE_DRIVE_TO_FIRST_LOAD_STATION";
        case STATE_DRIVE_TO_LOAD_STATION:
            return "STATE_DRIVE_TO_LOAD_STATION";
        case STATE_LOAD_CARGO:
            return "STATE_LOAD_CARGO";
        case STATE_FIND_UNLOAD_STATION:
            return "STATE_FIND_UNLOAD_STATION";
        case STATE_DRIVE_TO_UNLOAD_STATION:
            return "STATE_DRIVE_TO_UNLOAD_STATION";
        case STATE_POSITIONING:
        return "STATE_POSITIONING";
        case STATE_EXPLORATION:
            return "STATE_EXPLORATION";
        case STATE_UNLOAD_CARGO:
            return "STATE_UNLOAD_CARGO";
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

    ros::ServiceClient worldmodel_AddObject;

    int present_holzklotz;
    ros::Time time_holzklotz;

    void holzklotzCb(const std_msgs::Int32ConstPtr& holzklotz){
        if(present_holzklotz < 0 && holzklotz->data >= 0){
            present_holzklotz = holzklotz->data;
            time_holzklotz = ros::Time::now();
            ROS_INFO("Holzklotz inserted: %d", present_holzklotz);
        }else if(present_holzklotz >= 0 && holzklotz->data <= 0 && time_holzklotz < ros::Time::now() + ros::Duration(0.5)){
            ROS_INFO("Holzklotz removed: %d", present_holzklotz);
            present_holzklotz = -1;
            time_holzklotz = ros::Time::now();
        }else if(present_holzklotz != holzklotz->data){
            ROS_INFO("Holzklotz changed: %d to %d", present_holzklotz, holzklotz->data);
            present_holzklotz != holzklotz->data;
            time_holzklotz = ros::Time::now();
        }
    }


    void objectsCb(const hector_worldmodel_msgs::ObjectModelConstPtr& objectModel)
    {
    _number_objects = objectModel->objects.size();
    objects=objectModel;
    //ROS_INFO("in objects cb. Size: %d", _number_objects);

   /* std::string          new_id;
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
