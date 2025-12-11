# TPointCloud V1.1 header metadata

# Standard header
std_msgs/Header header

# Field from TPointCloudv11 header that are not included in PointCloud2
uint32 length   # this length is the size of the packet in bytes, including sizeof(TPointCloud_V1_1)
uint16 frame_counter
uint16 message_number
bool last_packet
uint32 bytes_per_target
SequenceType sequence_type
uint16 crd_count

# (length - sizeof(TPointCloud_V1_1)) / bytes_per_target
uint32 target_count