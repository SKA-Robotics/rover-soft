def deadzone(axis, deadzone):
    return axis if abs(axis) > deadzone else 0.0
