# Archived arm support

`servos.c` and `servos.h` contain the Feetech ST3215 arm implementation.
`ros_legacy.c` preserves the previous ROS interface, including arm commands
and servo telemetry. `robot_legacy.h` preserves its configuration.

These files are reference snapshots, excluded from CMake. The
`lib/pico_feetech` submodule is retained but is not added or linked by the
active build. Restoring arm support requires reintegrating its configuration,
startup and ROS declarations; the legacy ROS file also contains the retired
wheel-array and joint-state interface.
