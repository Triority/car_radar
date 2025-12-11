# Copyright (c) Sensrad 2023

# oden_interfaces/msg/FreeSpace

# A custom message type for Sensrad FreeSpace data

# Header for time and frame information
std_msgs/Header header

# Size of data
uint32 rows

# Actual data
float32[] azimuth
float32[] range

# Validity flag
bool is_valid