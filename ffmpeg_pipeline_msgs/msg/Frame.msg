std_msgs/Header header
# Common for video and audio
string format
uint8[] data
int64 pts
int64 pkt_dts
Rational time_base
# Audio only
int32 nb_samples
int32 sample_rate
string ch_layout
int64 duration
# Video only
int32 width
int32 height