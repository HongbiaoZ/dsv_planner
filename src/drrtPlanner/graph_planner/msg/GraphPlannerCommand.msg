uint8 command
uint8 COMMAND_DISABLE=0
uint8 COMMAND_GO_TO_ORIGIN=1
uint8 COMMAND_GO_TO_LOCATION=2

geometry_msgs/Point location # only relevant to COMMAND_GO_TO_LOCATION
