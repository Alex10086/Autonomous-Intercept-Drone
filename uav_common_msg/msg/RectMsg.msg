std_msgs/Header header

int32 x
int32 y
int32 depth
int32 width
int32 height
# YOLO or LightTrack score [0,1]; NaN for lost results, -1 if not provided.
float32 confidence -1.0
# Measurement provenance, not a calibrated confidence class.
uint8 UNKNOWN=0
uint8 YOLO=1
uint8 LIGHTTRACK=2
uint8 source

