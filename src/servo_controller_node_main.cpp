#include "servo_controller/servo_controller_node.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "servo_controller");
  servo_controller_node::ServoControllerNode node;
  ros::spin();
}
