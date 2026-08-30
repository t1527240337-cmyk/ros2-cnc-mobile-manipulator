uint8 IDLE=0
uint8 READY=1
uint8 PROCESSING=2
uint8 DONE=3
uint8 FAULT=4
uint8 HELD=5

std_msgs/Header header
string machine_id
uint8 state
bool door_open
bool part_present
string part_id
builtin_interfaces/Duration remaining_time
uint16 fault_code
