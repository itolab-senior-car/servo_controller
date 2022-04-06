#include "servo_controller/servo_controller.h"
#include <ros/ros.h>

namespace servo_controller
{
  auto limitServoAngle()
  {
    uint8_t angle = 0;
    return [angle](const uint8_t &in, const uint8_t &min,
                   const uint8_t &max) mutable -> uint8_t
           {
             angle = std::min(std::max(min, in), max);
	     return angle;
           };
  }

  auto lowpass_filter()
  {
    double result = 0.0;
    return [result](const double& in, const double& gain) mutable -> double
    {
      result = gain * result + ( 1 - gain) * in;
      return result;
    };
  }

  inline double radToDeg(const double &angle)
  {
    return angle * (180.0 / M_PI);
  }

  inline double ms2km(const double& lx)
  {
    return lx * 3.6;
  }

  Servo::Servo(const Configuration& config)
    : m_config(config)
  {
  }

  Twist Servo::smoothTwist(const Twist &twist_in) const
  {
    static auto lp_lx = lowpass_filter();
    static auto lp_az = lowpass_filter();

    Twist twist_out;
    twist_out.lx = lp_lx(twist_in.lx, 0.8);
    twist_out.az = lp_az(twist_in.az, 0.8);
    return twist_out;
  }

  inline uint8_t Servo::calcSteerAngle(const double &az) const
  {
    return std::ceil(static_cast<double>(m_config.mid_steering_angle * 2)
            - (M_PI_2 + az) * 180.0 / M_PI);
  }

  inline uint8_t Servo::calcAccel(const double& lx) const
  {
    auto speed = ms2km(lx);
    return (speed / 6.0) * m_config.max_accel_limit;
  }

  Control Servo::calcServo(const Twist& in) const
  {
    Control ctrl;
    ctrl.steering = calcSteerAngle(in.az);
    if (in.lx < 0)
    {
      ctrl.accel = calcAccel(-1.0* in.lx);
      ctrl.reverse = true;
    }
    else
    {
      ctrl.accel = calcAccel(in.lx);
      ctrl.reverse = false;
    }
    ctrl = calcServoOutput(ctrl);
    return ctrl;
  }

  Control Servo::calcServoOutput(const Control& in) const
  {
    static auto lim_steer = limitServoAngle();
    static auto lim_accel = limitServoAngle();
    uint8_t raw_steering = m_config.mid_steering_angle//前引く今を符号そのまま足す閾値値は頑張って出す;
    Control out;
    if (in.reverse == false){
      raw_steering = in.steering;
    }else{
      raw_steering = (m_config.mid_steering_angle - in.steering) + m_config.mid_steering_angle;
    }
    out.steering = lim_steer(raw_steering, m_config.min_steering_limit,
                             m_config.max_steering_limit);

    /*------------------------*/

    out.reverse = in.reverse;
    out.accel = lim_accel(in.accel, m_config.min_accel_limit, m_config.max_accel_limit);

    return out;
  }

  void Servo::setConfiguration(const Configuration &config)
  {
    m_config = config;
  }

  const Configuration& Servo::getConfiguration() const
  {
    return m_config;
  }
}
