#pragma once

// One thread, fixed order. main's WorkItem is queued on a WorkQueue thread
// and Run() takes no dt; the module computes dt from hrt. ScheduleNow() is
// what SubscriptionCallbackWorkItem calls when a topic is published.

namespace px4 {

class WorkQueue;

class WorkItem {
public:
  explicit WorkItem(const char *name) : _item_name(name) {}

  WorkItem(const WorkItem &) = delete;
  WorkItem &operator=(const WorkItem &) = delete;

  virtual ~WorkItem() = default;

  const char *ItemName() const { return _item_name; }

protected:
  friend class WorkQueue;
  virtual void Run(float dt) = 0;

private:
  const char *_item_name;
};

} // namespace px4
