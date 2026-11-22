# Vehicle thrust setpoint
#
# This is the thrust setpoint provided by the controller and fed into the control allocator.

uint64 timestamp        # [us] Time since system start

float32[3] xyz          # [-] [@range -1, 1] Thrust setpoint along X, Y, Z body axis. If set to NAN the motors affecting this axis are stopped.
