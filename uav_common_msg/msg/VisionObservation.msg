# One event per bbox, including rejected observations. Times below are ROS/system
# epoch seconds except image_stamp (original camera domain). Unavailable = NaN.
std_msgs/Header header
builtin_interfaces/Time image_stamp
float64 image_time_ros
float64 bbox_receive_time
float64 projection_time
# First velocity setpoint using this observation; NaN when never consumed.
float64 control_time
float64 attitude_before_time
float64 attitude_after_time
float64 image_to_bbox_ms
float64 image_to_control_ms
bool clock_valid
bool accepted
bool attitude_synced
uint8 RECEIVE_POSE=0
uint8 IMAGE_POSE=1
uint8 CLOCK_UNAVAILABLE=2
uint8 ATTITUDE_UNAVAILABLE=3
uint8 INVALID_BOX=4
uint8 NOT_GUIDING=5
uint8 OUT_OF_ORDER=6
uint8 ESTIMATOR_UNAVAILABLE=7
uint8 status
# Same enum as RectMsg.source: UNKNOWN=0, YOLO=1, LIGHTTRACK=2.
uint8 source
float32 confidence
int32 x
int32 y
int32 width
int32 height
# NED radians, azimuth atan2(North, East). Pure vision, never GPS guidance.
float64 los_v
float64 los_z
# Counterfactual using the receipt-pose snapshot, for paired P0 diagnostics.
float64 receive_pose_los_v
float64 receive_pose_los_z
# Raw image LOS above stays unchanged when prediction is enabled.
uint8 PIXEL_RAW=0
uint8 PIXEL_KF=1
uint8 estimator
float64 estimate_time_ros
# Fixed virtual-camera plane anchor used by the CV filter; NaN in raw mode.
float64 reference_time_ros
float64 estimated_u
float64 estimated_v
float64 estimated_du
float64 estimated_dv
float64 pixel_prediction_dt
float64 attitude_prediction_dt
# Actual LOS supplied to guidance; estimate epoch is explicit, not image epoch.
float64 control_los_v
float64 control_los_z
