# Copyright (c) Sensrad 2023
#
# oden_interfaces/msg/Detection
#
# A custom message type for a detection created from clustered data

float32 range
float32 azimuth
float32 elevation
float32 radial_speed
float32 power
float32 power_std
float32 doppler_std
float32[3] position
float32[9] covariance # Row major order from a 3x3 covariance matrix
uint8 type
float32 type_probability
uint32 label_id
int32 num_valid_detections
