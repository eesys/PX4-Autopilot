#include "uorb.hpp"

#include <cassert>
#include <cstdio>

namespace
{

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

void test_never_published()
{
	uorb::Subscription<NeverPublished> sub;
	NeverPublished out{42};

	assert(!sub.updated());
	assert(!sub.copy(out));
	assert(out.value == 42);
}

void test_one_publish()
{
	uorb::Subscription<OneShot> sub;
	uorb::Publication<OneShot> pub;

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

void test_two_publishes_keep_latest_and_gap()
{
	uorb::Subscription<TwoShots> sub;
	uorb::Publication<TwoShots> pub;

	assert(pub.publish(TwoShots{1}));
	assert(pub.publish(TwoShots{2}));
	assert(sub.updates_available() == 2);

	TwoShots out{};
	assert(sub.update(out));
	assert(out.value == 2);
	assert(sub.get_last_generation() == 2);
}

void test_update_without_new_publish()
{
	uorb::Subscription<NoNewUpdate> sub;
	uorb::Publication<NoNewUpdate> pub;

	assert(pub.publish(NoNewUpdate{3}));

	NoNewUpdate out{};
	assert(sub.update(out));
	assert(out.value == 3);

	NoNewUpdate again{9};
	assert(!sub.update(again));
	assert(again.value == 9);
}

void test_subscribers_track_generation_separately()
{
	uorb::Subscription<SharedTopic> first;
	uorb::Subscription<SharedTopic> second;
	uorb::Publication<SharedTopic> pub;

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
void test_late_subscriber_reads_latest_once()
{
	uorb::Publication<LateJoin> pub;
	assert(pub.publish(LateJoin{4}));
	assert(pub.publish(LateJoin{5}));

	uorb::Subscription<LateJoin> sub;
	assert(sub.updates_available() == 1);

	LateJoin out{};
	assert(sub.update(out));
	assert(out.value == 5);
	assert(!sub.update(out));
}

} // namespace

int main()
{
	test_never_published();
	test_one_publish();
	test_two_publishes_keep_latest_and_gap();
	test_update_without_new_publish();
	test_subscribers_track_generation_separately();
	test_late_subscriber_reads_latest_once();

	std::puts("uorb_test: ok");
	return 0;
}
