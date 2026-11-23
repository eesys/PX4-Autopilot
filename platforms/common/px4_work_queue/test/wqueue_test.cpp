#include "wqueue_test.h"

#include <px4_platform_common/px4_work_queue/WorkQueue.hpp>

#include <cassert>
#include <cstring>
#include <vector>

namespace {

class Probe : public px4::WorkItem {
public:
  Probe(const char *name, std::vector<const char *> &order, float &seen_dt)
      : WorkItem(name), _order(order), _seen_dt(seen_dt) {}

  void Run(float dt) override {
    _order.push_back(ItemName());
    _seen_dt = dt;
  }

private:
  std::vector<const char *> &_order;
  float &_seen_dt;
};

void test_runs_in_add_order_with_the_same_dt() {
  std::vector<const char *> order;
  float dt_plant = -1.f;
  float dt_attitude = -1.f;
  float dt_rate = -1.f;
  Probe plant("plant", order, dt_plant);
  Probe attitude("attitude", order, dt_attitude);
  Probe rate("rate", order, dt_rate);

  px4::WorkQueue queue;
  queue.Add(&plant);
  queue.Add(&attitude);
  queue.Run(0.01f);

  assert(order.size() == 2);
  assert(std::strcmp(order[0], "plant") == 0);
  assert(std::strcmp(order[1], "attitude") == 0);
  assert(dt_plant == 0.01f);
  assert(dt_attitude == 0.01f);

  queue.Add(&rate);
  queue.Run(0.02f);

  assert(order.size() == 5);
  assert(std::strcmp(order[2], "plant") == 0);
  assert(std::strcmp(order[3], "attitude") == 0);
  assert(std::strcmp(order[4], "rate") == 0);
  assert(dt_plant == 0.02f);
  assert(dt_attitude == 0.02f);
  assert(dt_rate == 0.02f);
}

} // namespace

int wqueue_test_main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  test_runs_in_add_order_with_the_same_dt();
  return 0;
}
