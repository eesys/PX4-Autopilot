#pragma once

// One process-wide DeviceNode per message type stands in for orb_metadata /
// ORB_ID. Generated topic headers do not emit ORB_DECLARE.

#include "uORBDeviceNode.hpp"

#include <type_traits>

namespace uORB {

template <typename T> DeviceNode &topic() {
  static_assert(std::is_trivially_copyable<T>::value,
                "topics are copied as bytes");
  static DeviceNode node(sizeof(T));
  return node;
}

} // namespace uORB
