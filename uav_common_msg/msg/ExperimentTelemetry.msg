# Algorithm observation only; file recording belongs to experiment_recorder.
std_msgs/Header header
string algorithm
# INIT, TAKEOFF, SEARCHING, INTERCEPT, TRACK_LOST, DONE
string state
# Actual node parameters serialized as a YAML mapping.
string parameters_yaml
bool guidance_valid
# NED angles, radians; azimuth follows atan2(North, East).
float64 los_v
float64 los_z
# Wrapped LOS increments consumed by PNG, radians/update (NOT radians/second).
float64 dlos_v
float64 dlos_z
float64 png_v
float64 png_z
# Optional solver diagnostics: NaN / empty status when unavailable.
float64 solver_time_ms
float64 cost
string solver_status
