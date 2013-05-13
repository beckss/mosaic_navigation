# Specifies the current state of the queue
Header header
Command next_cmd 

# state of queue
uint8 state
uint8 EMPTY     = 0   # Queue is empty
uint8 PAUSED    = 1   # Queue is paused
uint8 WAITING   = 2   # Queue is waiting for specified time
uint8 EXECUTING = 3   # Queue is enroute to next destination
