# Debt: main builds the px4_work_queue library from the .cpp files, then
# adds this test directory only when PX4_TESTING is set.

if(PX4_TESTING)
	add_subdirectory(test)
endif()
