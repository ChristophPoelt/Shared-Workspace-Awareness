import random
from globalVariables import currentTargetGlobal #not sure this line is necessary

TARGETS = ["position0", "position1", "position2"]

def selectNewTarget(currentTarget: str | None) -> str:
    """
    Select a random target different from the current one.
    If currentTarget is None, select any target.
    Also updates the global variable currentTargetGlobal.
    """
    if currentTarget is None:
        new_target = random.choice(TARGETS)
    else:
        if currentTarget not in TARGETS:
            raise ValueError(f"Unknown current target: {currentTarget}")
        available_targets = [t for t in TARGETS if t != currentTarget]
        new_target = random.choice(available_targets)

    # Update the global variable in another module
    currentTargetGlobal = new_target

    return new_target




#example usage
#current = "position1"
#next_target = selectNewTarget(current)