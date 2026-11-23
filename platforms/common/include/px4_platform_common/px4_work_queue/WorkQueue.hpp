#pragma once

// Run(float) walks items in Add() order on the calling thread.
// main's Run() is the worker loop: it waits, then pops whatever ScheduleNow()
// queued. There is no lock and no second thread here.

#include "WorkItem.hpp"

#include <vector>

namespace px4 {

class WorkQueue {
public:
  void Add(WorkItem *item) { _items.push_back(item); }

  void Run(float dt) {
    for (WorkItem *item : _items) {
      item->Run(dt);
    }
  }

private:
  std::vector<WorkItem *> _items;
};

} // namespace px4
