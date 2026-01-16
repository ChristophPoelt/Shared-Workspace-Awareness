from targetSelection import selectNewTarget
from globalVariables import currentTargetGlobal

#here we could handle the starting of all the nodes etc.

#init function that:
#subscribes to all needed nodes 
#starts all needed nodes
#-> Shiyi copies here already existing structure for the init function

#bring robot in inital position (drive to 0.0 and correct arm position)
#-> Franzi

#select an inital target
selectNewTarget

#do the target selected gesture -> Shiyi

#do the target indication gesture (point towards target) ->  Franzi

#if loop that handles all the voice commands and then 
#calls the according python script

#pseudo code: -> Christoph
#if abort command was detected:
#   call abort_command_logic
#if pause command was detected:
#   call pause_command_logic (needs to include doing the gesture)
#if whereAreYouGoing is detected:
#   call whereAreYouGoing_logic
#if continue is detected:
#   call continue_logic

#logic if target is reached -> we will figure it our later

#select the next target
selectNewTarget(currentTargetGlobal)