# Encodes the system state of the vehicle published by commander

uint64 timestamp # time since system start (microseconds)

uint8 arming_state
uint8 ARMING_STATE_DISARMED = 1
uint8 ARMING_STATE_ARMED    = 2
