# Best known radar state, reflecting most recent commands sent.

# Standard header
std_msgs/Header header

# In tx
bool tx_enabled

# Active sequence type
SequenceType active_seq

Thresholds thresholds

CFAR cfar

NTC ntc

SpotRsl spot_rsl

bool az_hist_enabled

bool range_hist_enabled

bool adt_enabled

bool local_max_enabled

# Last timestamp sent to radar
uint64 timestamp_ms
