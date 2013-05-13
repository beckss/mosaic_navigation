# Command.msg Used to specify commands to UAV

uint32 id          # unique sequence number for this command

uint8 type         # type of command - value must be one of following:
uint8 NULL     = 0 # Null command - do nothing
uint8 TAKEOFF  = 1 # Takeoff from current position (ignored if flying)
uint8 LAND     = 2 # Land at current position (ignored if landed)
uint8 GOTO_WP  = 3 # Goto specified GPS waypoint
uint8 GOTO_XY  = 4 # Goto specified waypoint

# At most one of the following should be specified, depending on type
GPSWaypoint gps_wp
CartesianWaypoint local_wp
