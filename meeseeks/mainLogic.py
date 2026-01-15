from targetSelection import selectNewTarget
from globalVariables import currentTargetGlobal

#here we could handle the starting of all the nodes etc.

#select an inital target
selectNewTarget

#do the target selected gesture
#do the target indication gesture (point towards target)

#if loop that handles all the voice commands and then 
#calls the according python script

#pseudo code:
#if abort command was detected:
#   call abort_command_logic.py
#if pause command was detected:
#   call pause_command_logic.py (needs to include doing the gesture)
#if whereAreYouGoing is detected:
#   call whereAreYouGoing_logic.py

#logic if target is reached

#select the next target
selectNewTarget(currentTargetGlobal)