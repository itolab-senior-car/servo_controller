#ifndef SERVO_CONTROLLER_SERVO_CONTROLLER_NODE_H
#define SERVO_CONTROLLER_SERVO_CONTROLLER_NODE_H

#include "servo_controller/servo_controller.h"
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include "itolab_senior_car_msgs/Servo.h"
#include "autoware_msgs/ControlCommandStamped.h"
#include "autoware_msgs/NDTStat.h"
#include "itolab_senior_car_msgs/DetectedObjectArray.h"
#include "itolab_senior_car_msgs/RegionObserver.h"

namespace servo_controller_node
{
  class ServoControllerNode
  {
  public:
    // ServoControllerNode();
    void Run();
  private:
    ros::NodeHandle nh;
    std::shared_ptr<servo_controller::Servo> servo_controller_ptr;
    int number_of_obstacle = 0;
    int stop_count {0};
    int count {0};
    double ndt_fitness_score = 0;
    bool stop_flag = false;
    bool shinkuma_flag = false;
    ros::Publisher servo_pub;
    ros::Subscriber twist_sub, ctrl_sub;
    ros::Subscriber cluster_sub;
    ros::Subscriber region_sub;
    ros::Subscriber ndt_stat_sub;
    ros::Subscriber shinkuma_spread_sub;
    //ros::Subscriber stop_sub;
    int test;

    int min_steering_servo_angle = 55;//1027_steering_servomin_65
    int mid_steering_servo_angle = 84;//1027_steering_servomidle_85
    int max_steering_servo_angle = 105;//1027_steering_servomax_101
    int min_accel_servo_angle = 0;
    int max_accel_servo_angle = 50;

    void ndtStatCallback(const autoware_msgs::NDTStatConstPtr& msg);
    void pointClusterCallback(const itolab_senior_car_msgs::DetectedObjectArrayConstPtr& msg);
    void twistCmdCallback(const geometry_msgs::TwistStampedConstPtr& msg);
    void regionCallback(const itolab_senior_car_msgs::RegionObserverConstPtr& msg);
    void ctrlCmdCallback(const autoware_msgs::ControlCommandStampedConstPtr& msg);
    void shinkumaCallback(const std_msgs::BoolPtr& msg);
    //void closestCallback(const std_msgs::BoolConstPtr& msg);
    void checkTwist(const servo_controller::Twist, const servo_controller::Twist twist_prev, const double& dt);
  };
}

#endif // SERVO_CONTROLLER_SERVO_CONTROLLER_H





