#include "sih.hpp"

#include <px4_platform_common/px4_work_queue/WorkQueue.hpp>

#include <cassert>
#include <cmath>
#include <cstdio>

namespace {

float dot3(const float a[3], const float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void quat_mul(const float a[4], const float b[4], float out[4]) {
  out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

void quat_normalize(float q[4]) {
  const float n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] /= n;
  q[1] /= n;
  q[2] /= n;
  q[3] /= n;
}

// Unit quaternion, body vector to NED.
void rotate_body_to_ned(const float q[4], const float v[3], float out[3]) {
  const float tx = 2.f * (q[2] * v[2] - q[3] * v[1]);
  const float ty = 2.f * (q[3] * v[0] - q[1] * v[2]);
  const float tz = 2.f * (q[1] * v[1] - q[2] * v[0]);
  out[0] = v[0] + q[0] * tx + (q[2] * tz - q[3] * ty);
  out[1] = v[1] + q[0] * ty + (q[3] * tx - q[1] * tz);
  out[2] = v[2] + q[0] * tz + (q[1] * ty - q[2] * tx);
}

void axis_angle_to_quat(const float w[3], float dt, float dq[4]) {
  const float angle = std::sqrt(dot3(w, w)) * dt;
  if (angle < 1.e-8f) {
    dq[0] = 1.f;
    dq[1] = 0.f;
    dq[2] = 0.f;
    dq[3] = 0.f;
    return;
  }

  const float half = 0.5f * angle;
  const float scale = std::sin(half) / (angle / dt);
  dq[0] = std::cos(half);
  dq[1] = w[0] * scale;
  dq[2] = w[1] * scale;
  dq[3] = w[2] * scale;
}

} // namespace

Sih::Sih() : px4::WorkItem("sih") {}

void Sih::Run(float dt) {
  vehicle_torque_setpoint_s torque{};
  if (_torque_sub.update(torque)) {
    _torque[0] = torque.xyz[0];
    _torque[1] = torque.xyz[1];
    _torque[2] = torque.xyz[2];
  }

  vehicle_thrust_setpoint_s thrust{};
  if (_thrust_sub.update(thrust)) {
    _thrust_body[0] = thrust.xyz[0];
    _thrust_body[1] = thrust.xyz[1];
    _thrust_body[2] = thrust.xyz[2];
  }

  integrate(dt);
  _time_us += static_cast<uint64_t>(static_cast<double>(dt) * 1.e6);
  publish();
}

void Sih::integrate(float dt) {
  // Mass and inertia are 1. Use the attitude and rate from the start of the step.
  float accel_ned[3];
  rotate_body_to_ned(_q, _thrust_body, accel_ned);
  _v_ned[0] += accel_ned[0] * dt;
  _v_ned[1] += accel_ned[1] * dt;
  _v_ned[2] += accel_ned[2] * dt;

  float dq[4];
  axis_angle_to_quat(_w_body, dt, dq);
  float q_next[4];
  quat_mul(_q, dq, q_next);
  quat_normalize(q_next);
  _q[0] = q_next[0];
  _q[1] = q_next[1];
  _q[2] = q_next[2];
  _q[3] = q_next[3];

  _w_body[0] += _torque[0] * dt;
  _w_body[1] += _torque[1] * dt;
  _w_body[2] += _torque[2] * dt;
}

void Sih::publish() {
  vehicle_attitude_s attitude{};
  attitude.timestamp = _time_us;
  attitude.q[0] = _q[0];
  attitude.q[1] = _q[1];
  attitude.q[2] = _q[2];
  attitude.q[3] = _q[3];
  _attitude_pub.publish(attitude);

  vehicle_angular_velocity_s rate{};
  rate.timestamp = _time_us;
  rate.xyz[0] = _w_body[0];
  rate.xyz[1] = _w_body[1];
  rate.xyz[2] = _w_body[2];
  _rate_pub.publish(rate);
}

namespace {

void test_constant_roll_torque_increases_rate() {
  Sih sih;
  px4::WorkQueue queue;
  queue.Add(&sih);

  uORB::Publication<vehicle_torque_setpoint_s> torque_pub;
  vehicle_torque_setpoint_s torque{};
  torque.xyz[0] = 1.f;
  assert(torque_pub.publish(torque));

  // Upward thrust is negative Z in FRD.
  uORB::Publication<vehicle_thrust_setpoint_s> thrust_pub;
  vehicle_thrust_setpoint_s thrust{};
  thrust.xyz[2] = -1.f;
  assert(thrust_pub.publish(thrust));

  uORB::Subscription<vehicle_angular_velocity_s> rate_sub;
  uORB::Subscription<vehicle_attitude_s> attitude_sub;

  queue.Run(0.2f);
  vehicle_angular_velocity_s rate{};
  assert(rate_sub.update(rate));
  const float rate_after_one = rate.xyz[0];
  assert(rate_after_one > 0.f);
  assert(rate.xyz[1] == 0.f);
  assert(rate.xyz[2] == 0.f);

  queue.Run(0.2f);
  assert(rate_sub.update(rate));
  assert(rate.xyz[0] > rate_after_one);

  vehicle_attitude_s attitude{};
  assert(attitude_sub.update(attitude));
  assert(attitude.q[1] > 0.f);
  assert(attitude.q[0] < 1.f);

  // Upward acceleration is negative NED Z while the body is still near level.
  assert(sih.velocity_ned()[2] < 0.f);
}

} // namespace

int main() {
  test_constant_roll_torque_increases_rate();
  std::puts("PASS");
  return 0;
}
