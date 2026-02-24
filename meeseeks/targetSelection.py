import random

TARGETS = ["position0", "position1", "position2"]

def selectNewTarget(currentTarget: str | None) -> str:
    """
    Select a random target different from the current one.
    If currentTarget is None, select any target.
    Returns the chosen target; callers decide when/how to publish or persist it.
    """
    if currentTarget is None:
        new_target = random.choice(TARGETS)
    else:
        if currentTarget not in TARGETS:
            raise ValueError(f"Unknown current target: {currentTarget}")
        available_targets = [t for t in TARGETS if t != currentTarget]
        new_target = random.choice(available_targets)

    return new_target




#example usage
#current = "position1"
#next_target = selectNewTarget(current)
