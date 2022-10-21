float32[6] efforts  # amps
float32[6] vels     # rad/s
float32[6] pos      # rad
time startSyncTime 
uint32 isrTicks # this would overflow if the robot is left on for 497 days straight at 100 hz 
uint8 bufferHealth