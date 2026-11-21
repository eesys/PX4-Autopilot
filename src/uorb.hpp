#pragma once

// Single-threaded uORB with queue depth 1.
//
// Studied from the upstream checkout (PX4-Autopilot):
//   platforms/common/uORB/uORB.h
//   platforms/common/uORB/Publication.hpp
//   platforms/common/uORB/Subscription.hpp
//   platforms/common/uORB/uORBDeviceNode.hpp
//   platforms/common/uORB/uORBDeviceNode.cpp
//   platforms/common/uORB/uORBManager.cpp
//
// One process-wide node per message type stands in for orb_metadata.
// publish() copies the struct as bytes and bumps a monotonic generation.
// A subscriber that falls behind still reads the latest sample; the gap
// stays visible as updates_available(). See ROADMAP.md for what this
// step leaves out (multi-instance, queues, callbacks, atomics).

#include <cstddef>
#include <cstring>
#include <type_traits>
#include <vector>

namespace uorb
{

class TopicNode
{
public:
	explicit TopicNode(std::size_t size) : _data(size) {}

	void publish(const void *src)
	{
		std::memcpy(_data.data(), src, _data.size());
		_generation += 1;
		_published = true;
	}

	// Queue depth 1: always the latest sample, generation jumps to current.
	// Same contract as DeviceNode::copy() when o_queue == 1.
	bool copy(void *dst, unsigned &generation) const
	{
		if (!_published || (dst == nullptr)) {
			return false;
		}

		std::memcpy(dst, _data.data(), _data.size());
		generation = _generation;
		return true;
	}

	unsigned generation() const { return _generation; }

	unsigned updates_available(unsigned generation) const
	{
		return _generation - generation;
	}

	// DeviceNode::get_initial_generation(): a late subscriber may read the
	// sample already stored, and counts that as a single pending update.
	unsigned initial_generation() const
	{
		return _generation - (_published ? 1u : 0u);
	}

	bool published() const { return _published; }

private:
	std::vector<unsigned char> _data;
	unsigned _generation{0};
	bool _published{false};
};

template<typename T>
TopicNode &topic()
{
	static_assert(std::is_trivially_copyable<T>::value, "topics are copied as bytes");
	static TopicNode node(sizeof(T));
	return node;
}

template<typename T>
class Publication
{
public:
	bool publish(const T &data)
	{
		topic<T>().publish(&data);
		return true;
	}
};

template<typename T>
class Subscription
{
public:
	Subscription() : _last_generation(topic<T>().initial_generation()) {}

	bool updated() const
	{
		return topic<T>().updates_available(_last_generation) > 0;
	}

	// Copies only when a publish has happened since the last successful read.
	bool update(T &dst)
	{
		if (!updated()) {
			return false;
		}

		return topic<T>().copy(&dst, _last_generation);
	}

	// Returns the latest sample once anything has been published, and catches
	// this subscriber up. orb_data_copy(..., only_if_updated=false).
	bool copy(T &dst)
	{
		if (!topic<T>().published()) {
			return false;
		}

		return topic<T>().copy(&dst, _last_generation);
	}

	unsigned get_last_generation() const { return _last_generation; }

	unsigned updates_available() const
	{
		return topic<T>().updates_available(_last_generation);
	}

private:
	unsigned _last_generation;
};

} // namespace uorb
