#pragma once

#include "uORB.h"

namespace uORB {

template <typename T> class Subscription {
public:
  Subscription() : _last_generation(topic<T>().get_initial_generation()) {}

  bool updated() const {
    return topic<T>().updates_available(_last_generation) > 0;
  }

  // orb_data_copy(..., only_if_updated=true)
  bool update(T &dst) {
    if (!updated()) {
      return false;
    }

    return topic<T>().copy(&dst, _last_generation);
  }

  // orb_data_copy(..., only_if_updated=false): latest sample, then catch up.
  bool copy(T &dst) { return topic<T>().copy(&dst, _last_generation); }

  unsigned get_last_generation() const { return _last_generation; }

  unsigned updates_available() const {
    return topic<T>().updates_available(_last_generation);
  }

private:
  unsigned _last_generation;
};

} // namespace uORB
