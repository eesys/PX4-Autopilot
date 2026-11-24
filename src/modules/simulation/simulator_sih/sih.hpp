#pragma once

// Rigid body for the attitude loop. main's Sih also integrates position,
// gravity, aero, and the w x Iw term, and it is a ModuleBase, not a WorkItem.

#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

class Sih : public px4::WorkItem {
public:
  Sih();

  // NED velocity from integrating thrust. Not a uORB topic in this step.
  const float *velocity_ned() const { return _v_ned; }

private:
  void Run(float dt) override;
  void integrate(float dt);
  void publish();

  // q rotates a body-frame vector into NED. w, x, y, z.
  float _q[4]{1.f, 0.f, 0.f, 0.f};
  float _w_body[3]{};
  float _v_ned[3]{};
  float _torque[3]{};
  float _thrust_body[3]{};
  uint64_t _time_us{0};

  uORB::Subscription<vehicle_torque_setpoint_s> _torque_sub{};
  uORB::Subscription<vehicle_thrust_setpoint_s> _thrust_sub{};
  uORB::Publication<vehicle_attitude_s> _attitude_pub{};
  uORB::Publication<vehicle_angular_velocity_s> _rate_pub{};
};
