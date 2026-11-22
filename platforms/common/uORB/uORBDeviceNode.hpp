#pragma once

// Queue depth 1, one thread. main's DeviceNode also inherits cdev::CDev and
// covers o_queue > 1, atomics, and the node lock. write()'s filp is omitted.

#include <cstddef>
#include <cstring>
#include <vector>

namespace uORB {

class DeviceNode {
public:
  explicit DeviceNode(std::size_t size) : _data(size) {}

  DeviceNode(const DeviceNode &) = delete;
  DeviceNode &operator=(const DeviceNode &) = delete;

  // write(): copy the sample and bump generation. fetch_add's old value is the
  // slot index; depth 1 always uses slot 0, so only the new count is kept.
  int publish_data(const void *data) {
    std::memcpy(_data.data(), data, _data.size());
    _generation += 1;
    _data_valid = true;
    return 0;
  }

  // DeviceNode::copy() when o_queue == 1.
  bool copy(void *dst, unsigned &generation) const {
    if (!_data_valid || (dst == nullptr)) {
      return false;
    }

    std::memcpy(dst, _data.data(), _data.size());
    generation = _generation;
    return true;
  }

  unsigned updates_available(unsigned generation) const {
    return _generation - generation;
  }

  // get_initial_generation(): a late subscriber may read the stored sample.
  unsigned get_initial_generation() const {
    return _generation - (_data_valid ? 1u : 0u);
  }

private:
  std::vector<unsigned char> _data;
  unsigned _generation{0};
  bool _data_valid{false};
};

} // namespace uORB
