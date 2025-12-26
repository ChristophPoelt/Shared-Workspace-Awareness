import random

TARGETS = ["position0", "position1", "position2"]


def selectNewTarget(currentTarget: str) -> str:
    """
    Select a random target different from the current one.
    Returns a String with the position name
    """
    if currentTarget not in TARGETS:
        raise ValueError(f"Unknown current target: {currentTarget}")

    #creates a list with all targets in t except the current one
    available_targets = [t for t in TARGETS if t != currentTarget]
    return random.choice(available_targets)


#example usage
#current = "position1"
#next_target = selectNewTarget(current)