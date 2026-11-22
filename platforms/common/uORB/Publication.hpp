#pragma once

#include "uORB.h"

namespace uORB {

template <typename T> class Publication {
public:
  bool publish(const T &data) { return topic<T>().publish_data(&data) == 0; }
};

} // namespace uORB
