# Defines which controllers should run

uint64 timestamp		# time since system start (microseconds)
bool flag_armed			# synonym for actuator_armed.armed

bool flag_control_attitude_enabled		# true if attitude stabilization is mixed in
bool flag_control_rates_enabled			# true if rates are stabilized
bool flag_control_allocation_enabled		# true if control allocation is enabled
