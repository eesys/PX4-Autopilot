# Debt: main also builds wqueue_scheduled_test.cpp and wqueue_start.cpp, and
# links the px4_work_queue library. This copy runs the fixed-step test on the host.

px4_add_module(
	MODULE lib__work_queue__test__wqueue_test
	MAIN wqueue_test
	SRCS
		wqueue_main.cpp
		wqueue_test.cpp
	)
