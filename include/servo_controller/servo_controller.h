#ifndef SERVO_CONTROLLER_SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_SERVO_CONTROLLER_H
#include <algorithm>
#include <cmath>
#include <boost/optional.hpp>

namespace servo_controller
{
  struct Twist
  {
    double lx = 0; // lateral acceleration x direction
    double az = 0; // angular velocity z direction, yaw
  };

  typedef struct
  {
    uint8_t mid_steering_angle;
    uint8_t min_steering_limit;
    uint8_t max_steering_limit;
    uint8_t min_accel_limit;
    uint8_t max_accel_limit;
    double wheelbase = 1.2;
  }
  Configuration;

  inline double radToDeg(const double&);

  struct Control
  {
    uint8_t steering = 0;
    uint8_t accel = 0;
    uint8_t reverse = 0;
  };

  class Servo
  {
  public:
    explicit Servo(const Configuration& config);
    Twist smoothTwist(const Twist&) const;
    
    inline uint8_t calcSteerAngle(const double&) const;
    uint8_t calcAccel(const double&) const;
    Control calcServo(const Twist&) const;
    Control calcServoOutput(const Control&) const;

    void setConfiguration(const Configuration& config);
    const Configuration& getConfiguration() const;
  private:
    Configuration m_config;
  };
}
#endif //SERVO_CONTROLLER_SERVO_CONTROLLER_H
