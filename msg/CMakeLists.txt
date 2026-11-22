# Debt: main lists every message, runs the EmPy generator, and builds uorb_msgs.
# This copy lists the nine closed-loop messages and emits their headers only.

find_program(PYTHON_EXECUTABLE python3 REQUIRED)

set(msg_files
	versioned/VehicleAttitude.msg
	versioned/VehicleAttitudeSetpoint.msg
	versioned/VehicleAngularVelocity.msg
	versioned/VehicleRatesSetpoint.msg
	VehicleTorqueSetpoint.msg
	VehicleThrustSetpoint.msg
	versioned/ActuatorMotors.msg
	versioned/VehicleStatus.msg
	VehicleControlMode.msg
)

set(msg_out_path ${CMAKE_BINARY_DIR}/uORB/topics)

set(uorb_headers)
set(msg_abs)
foreach(msg_file ${msg_files})
	get_filename_component(msg ${msg_file} NAME_WE)
	string(REGEX REPLACE "(.)([A-Z][a-z]+)" "\\1_\\2" msg "${msg}")
	string(REGEX REPLACE "([a-z0-9])([A-Z])" "\\1_\\2" msg "${msg}")
	string(TOLOWER "${msg}" msg)
	list(APPEND uorb_headers ${msg_out_path}/${msg}.h)
	list(APPEND msg_abs ${CMAKE_CURRENT_SOURCE_DIR}/${msg_file})
endforeach()

add_custom_command(
	OUTPUT ${uorb_headers}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		--headers
		-f ${msg_abs}
		-o ${msg_out_path}
	DEPENDS
		${msg_abs}
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
	COMMENT "Generating uORB topic headers"
	VERBATIM
)
add_custom_target(uorb_headers DEPENDS ${uorb_headers})
