#include "servo_controller/servo_controller_node.h"
#include <std_msgs/Bool.h>

namespace servo_controller_node
{
  void ServoControllerNode::Run()
  {
    ndt_stat_sub = nh.subscribe("ndt_stat", 1, &ServoControllerNode::ndtStatCallback, this);
    cluster_sub = nh.subscribe("lidar_detected_object", 1, &ServoControllerNode::pointClusterCallback, this);
    twist_sub = nh.subscribe("twist_raw", 1, &ServoControllerNode::twistCmdCallback, this);
    region_sub = nh.subscribe("region_information", 1, &ServoControllerNode::regionCallback, this);
    // stop_sub = nh.subscribe("judge_closest", 1, &ServoControllerNode::closestCallback, this);
    servo_pub = nh.advertise<itolab_senior_car_msgs::Servo>("servo_cmd", 5);
    shinkuma_spread_sub = nh.subscribe("shinkuma_spread", 1, &ServoControllerNode::shinkumaCallback, this);

    ros::NodeHandle private_nh("~");
    private_nh.getParam("min_steering_servo_angle", min_steering_servo_angle);
    private_nh.getParam("mid_steering_servo_angle", mid_steering_servo_angle);
    private_nh.getParam("max_steering_servo_angle", max_steering_servo_angle);
    private_nh.getParam("min_accel_servo_angle", min_accel_servo_angle);
    private_nh.getParam("max_accel_servo_angle", max_accel_servo_angle);

    ROS_INFO("min_steering_servo_angle is %d", min_steering_servo_angle);
    ROS_INFO("mid_steering_servo_angle is %d", mid_steering_servo_angle);
    ROS_INFO("max_steering_servo_angle is %d", max_steering_servo_angle);
    ROS_INFO("min_accel_servo_angle is %d", min_accel_servo_angle);
    ROS_INFO("max_accel_servo_angle is %d", max_accel_servo_angle);


    servo_controller::Configuration config;
    config.min_steering_limit = static_cast<uint8_t>(min_steering_servo_angle);
    config.mid_steering_angle = static_cast<uint8_t>(mid_steering_servo_angle);
    config.max_steering_limit = static_cast<uint8_t>(max_steering_servo_angle);
    config.min_accel_limit = static_cast<uint8_t>(min_accel_servo_angle);
    config.max_accel_limit = static_cast<uint8_t>(max_accel_servo_angle);

    servo_controller_ptr = std::make_shared<servo_controller::Servo>(config);
  }

  void ServoControllerNode::ndtStatCallback(const autoware_msgs::NDTStatConstPtr& msg)
  {
    ndt_fitness_score = msg->score;
  }

  void ServoControllerNode::pointClusterCallback(const itolab_senior_car_msgs::DetectedObjectArrayConstPtr&  msg)
  {
    number_of_obstacle = msg->objects.size();
  }

  //void ServoControllerNode::closestCallback(const std_msgs::BoolConstPtr &msg){
  //  stop_flag = msg->data;
  //}

  void ServoControllerNode::twistCmdCallback(const geometry_msgs::TwistStampedConstPtr& msg)
  {
    std::cout << "There is(are) " << number_of_obstacle << " blocking Senior Car\n";
    std::cout << "NDT Fitness Score is " << ndt_fitness_score << "\n";
    std::cout << "shinkuma_flag is " << shinkuma_flag << std::endl;

    const servo_controller::Twist twist  = {msg->twist.linear.x, msg->twist.angular.z};
    auto twist_smoothed = servo_controller_ptr->smoothTwist(twist);
    auto servo_out = servo_controller_ptr->calcServo(twist_smoothed);
    itolab_senior_car_msgs::Servo out;
    out.steering = servo_out.steering;
        ros::Rate r(0.5);

    if(stop_count == 1 || shinkuma_flag == 1){
      out.accel = 0;
      // ROS_INFO("idx %ld",region_idx);
      servo_pub.publish(out);
      r.sleep();
      stop_count = 2;
    }else{
      if (ndt_fitness_score < 1000){
        r.reset();
        out.accel = servo_out.accel * (number_of_obstacle == 0);

        out.reverse = servo_out.reverse;
        servo_pub.publish(out);
      }else{
        ROS_ERROR("NDT matching is flying!");
        out.steering = mid_steering_servo_angle;
        out.accel = 0;
        servo_pub.publish(out);
      }
    }
   
    ROS_INFO("stop count %d",stop_count);


  }

  void ServoControllerNode::regionCallback(const itolab_senior_car_msgs::RegionObserverConstPtr& msg)
  {
    bool isWithin = msg->is_within;
    if (isWithin == true && count == 0)
    {
      stop_count = 1;
      count++;
    }

  }

  void ServoControllerNode::shinkumaCallback(const std_msgs::BoolPtr& msg)
  {
    shinkuma_flag = msg->data;
  }
}
