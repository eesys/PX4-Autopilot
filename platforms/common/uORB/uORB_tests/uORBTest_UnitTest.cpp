#include "uORBTest_UnitTest.hpp"

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

#include <cassert>
#include <cmath>
#include <type_traits>

namespace {

struct NeverPublished {
  int value;
};

struct OneShot {
  int value;
};

struct TwoShots {
  int value;
};

struct NoNewUpdate {
  int value;
};

struct SharedTopic {
  int value;
};

struct LateJoin {
  int value;
};

void test_never_published() {
  uORB::Subscription<NeverPublished> sub;
  NeverPublished out{42};

  assert(!sub.updated());
  assert(!sub.copy(out));
  assert(out.value == 42);
}

void test_one_publish() {
  uORB::Subscription<OneShot> sub;
  uORB::Publication<OneShot> pub;

  assert(pub.publish(OneShot{7}));

  OneShot out{};
  assert(sub.update(out));
  assert(out.value == 7);
  assert(sub.get_last_generation() == 1);

  OneShot again{};
  assert(sub.copy(again));
  assert(again.value == 7);
  assert(!sub.updated());
}

void test_two_publishes_keep_latest_and_gap() {
  uORB::Subscription<TwoShots> sub;
  uORB::Publication<TwoShots> pub;

  assert(pub.publish(TwoShots{1}));
  assert(pub.publish(TwoShots{2}));
  assert(sub.updates_available() == 2);

  TwoShots out{};
  assert(sub.update(out));
  assert(out.value == 2);
  assert(sub.get_last_generation() == 2);
}

void test_update_without_new_publish() {
  uORB::Subscription<NoNewUpdate> sub;
  uORB::Publication<NoNewUpdate> pub;

  assert(pub.publish(NoNewUpdate{3}));

  NoNewUpdate out{};
  assert(sub.update(out));
  assert(out.value == 3);

  NoNewUpdate again{9};
  assert(!sub.update(again));
  assert(again.value == 9);
}

void test_subscribers_track_generation_separately() {
  uORB::Subscription<SharedTopic> first;
  uORB::Subscription<SharedTopic> second;
  uORB::Publication<SharedTopic> pub;

  assert(pub.publish(SharedTopic{1}));
  assert(pub.publish(SharedTopic{2}));
  assert(first.updates_available() == 2);
  assert(second.updates_available() == 2);

  SharedTopic out{};
  assert(first.update(out));
  assert(out.value == 2);
  assert(first.get_last_generation() == 2);
  assert(!first.updated());

  assert(second.updates_available() == 2);
  assert(second.get_last_generation() == 0);
}

// get_initial_generation(): created after two publishes, one pending sample.
void test_late_subscriber_reads_latest_once() {
  uORB::Publication<LateJoin> pub;
  assert(pub.publish(LateJoin{4}));
  assert(pub.publish(LateJoin{5}));

  uORB::Subscription<LateJoin> sub;
  assert(sub.updates_available() == 1);

  LateJoin out{};
  assert(sub.update(out));
  assert(out.value == 5);
  assert(!sub.update(out));
}

void test_closed_loop_messages() {
  static_assert(std::is_same<px4::msg::VehicleAttitude, vehicle_attitude_s>::value,
                "topic header alias");
  static_assert(sizeof(vehicle_attitude_s::q) == 4 * sizeof(float), "q is wxyz");
  static_assert(vehicle_status_s::ARMING_STATE_DISARMED == 1, "");
  static_assert(vehicle_status_s::ARMING_STATE_ARMED == 2, "");
  static_assert(actuator_motors_s::NUM_CONTROLS == 12, "");
  static_assert(sizeof(actuator_motors_s::control) / sizeof(float) ==
                    actuator_motors_s::NUM_CONTROLS,
                "");

  // Hamilton (w, x, y, z). Identity: FRD aligned with NED.
  uORB::Subscription<vehicle_attitude_s> attitude_sub;
  uORB::Publication<vehicle_attitude_s> attitude_pub;
  vehicle_attitude_s attitude{};
  attitude.timestamp = 10;
  attitude.q[0] = 1.f;
  assert(attitude_pub.publish(attitude));
  vehicle_attitude_s attitude_out{};
  assert(attitude_sub.update(attitude_out));
  assert(attitude_out.timestamp == 10);
  assert(attitude_out.q[0] == 1.f);
  assert(attitude_out.q[1] == 0.f);
  assert(attitude_out.q[2] == 0.f);
  assert(attitude_out.q[3] == 0.f);

  uORB::Subscription<vehicle_attitude_setpoint_s> attitude_sp_sub;
  uORB::Publication<vehicle_attitude_setpoint_s> attitude_sp_pub;
  vehicle_attitude_setpoint_s attitude_sp{};
  attitude_sp.q_d[0] = 1.f;
  attitude_sp.thrust_body[2] = -0.5f;
  assert(attitude_sp_pub.publish(attitude_sp));
  vehicle_attitude_setpoint_s attitude_sp_out{};
  assert(attitude_sp_sub.update(attitude_sp_out));
  assert(attitude_sp_out.q_d[0] == 1.f);
  assert(attitude_sp_out.thrust_body[2] == -0.5f);

  uORB::Subscription<vehicle_angular_velocity_s> rate_sub;
  uORB::Publication<vehicle_angular_velocity_s> rate_pub;
  vehicle_angular_velocity_s rate{};
  rate.xyz[0] = 0.1f;
  assert(rate_pub.publish(rate));
  vehicle_angular_velocity_s rate_out{};
  assert(rate_sub.update(rate_out));
  assert(rate_out.xyz[0] == 0.1f);

  uORB::Subscription<vehicle_rates_setpoint_s> rate_sp_sub;
  uORB::Publication<vehicle_rates_setpoint_s> rate_sp_pub;
  vehicle_rates_setpoint_s rate_sp{};
  rate_sp.roll = 0.2f;
  rate_sp.thrust_body[2] = -0.5f;
  assert(rate_sp_pub.publish(rate_sp));
  vehicle_rates_setpoint_s rate_sp_out{};
  assert(rate_sp_sub.update(rate_sp_out));
  assert(rate_sp_out.roll == 0.2f);
  assert(rate_sp_out.thrust_body[2] == -0.5f);

  uORB::Subscription<vehicle_torque_setpoint_s> torque_sub;
  uORB::Publication<vehicle_torque_setpoint_s> torque_pub;
  vehicle_torque_setpoint_s torque{};
  torque.xyz[0] = 0.3f;
  assert(torque_pub.publish(torque));
  vehicle_torque_setpoint_s torque_out{};
  assert(torque_sub.update(torque_out));
  assert(torque_out.xyz[0] == 0.3f);

  uORB::Subscription<vehicle_thrust_setpoint_s> thrust_sub;
  uORB::Publication<vehicle_thrust_setpoint_s> thrust_pub;
  vehicle_thrust_setpoint_s thrust{};
  thrust.xyz[2] = -0.4f;
  assert(thrust_pub.publish(thrust));
  vehicle_thrust_setpoint_s thrust_out{};
  assert(thrust_sub.update(thrust_out));
  assert(thrust_out.xyz[2] == -0.4f);

  uORB::Subscription<actuator_motors_s> motors_sub;
  uORB::Publication<actuator_motors_s> motors_pub;
  actuator_motors_s motors{};
  motors.control[0] = NAN;
  motors.control[1] = 0.25f;
  assert(motors_pub.publish(motors));
  actuator_motors_s motors_out{};
  assert(motors_sub.update(motors_out));
  assert(std::isnan(motors_out.control[0]));
  assert(motors_out.control[1] == 0.25f);

  uORB::Subscription<vehicle_status_s> status_sub;
  uORB::Publication<vehicle_status_s> status_pub;
  vehicle_status_s status{};
  status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
  assert(status_pub.publish(status));
  vehicle_status_s status_out{};
  assert(status_sub.update(status_out));
  assert(status_out.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

  uORB::Subscription<vehicle_control_mode_s> mode_sub;
  uORB::Publication<vehicle_control_mode_s> mode_pub;
  vehicle_control_mode_s mode{};
  mode.flag_armed = true;
  mode.flag_control_attitude_enabled = true;
  mode.flag_control_rates_enabled = true;
  mode.flag_control_allocation_enabled = true;
  assert(mode_pub.publish(mode));
  vehicle_control_mode_s mode_out{};
  assert(mode_sub.update(mode_out));
  assert(mode_out.flag_armed);
  assert(mode_out.flag_control_attitude_enabled);
  assert(mode_out.flag_control_rates_enabled);
  assert(mode_out.flag_control_allocation_enabled);
}

} // namespace

uORBTest::UnitTest &uORBTest::UnitTest::instance() {
  static uORBTest::UnitTest t;
  return t;
}

int uORBTest::UnitTest::test() {
  test_never_published();
  test_one_publish();
  test_two_publishes_keep_latest_and_gap();
  test_update_without_new_publish();
  test_subscribers_track_generation_separately();
  test_late_subscriber_reads_latest_once();
  test_closed_loop_messages();
  return 0;
}
