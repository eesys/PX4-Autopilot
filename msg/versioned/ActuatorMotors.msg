# Motor control message
#
# Normalised thrust setpoint for up to 12 motors.
# Published by the vehicle's allocation and consumed by the ESC protocol drivers e.g. PWM, DSHOT, UAVCAN.

uint64 timestamp # [us] Time since system start

uint8 NUM_CONTROLS = 12 # output_functions.yaml Motor.count
float32[12] control # [@range -1, 1] Normalized thrust. Where 1 means maximum positive thrust, -1 maximum negative (if not supported by the output, <0 maps to NaN). NaN maps to disarmed (stop the motors)
