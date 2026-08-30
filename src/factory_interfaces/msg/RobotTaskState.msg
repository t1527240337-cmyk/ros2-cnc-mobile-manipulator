uint8 KIND_UNLOAD_FINISHED=0
uint8 KIND_LOAD_RAW=1
uint8 KIND_DOCK_AND_CHARGE=2

uint8 STATUS_PENDING=0
uint8 STATUS_RESERVED=1
uint8 STATUS_RUNNING=2
uint8 STATUS_SUCCEEDED=3
uint8 STATUS_FAILED=4
uint8 STATUS_CANCELED=5

string task_id
uint8 kind
uint8 priority
string machine_id
string part_id
string order_id
string source_event_id
string[] depends_on
uint8 status
uint32 attempts
uint64 created_sequence
string detail
string last_phase
string last_feedback
string reconciliation_note
