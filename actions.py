from enum import IntEnum, unique

@unique
class Actions(IntEnum):
    STAY = 0
    MOVE_E = 1
    MOVE_NE = 2
    MOVE_N = 3
    MOVE_NW = 4
    MOVE_W = 5
    MOVE_SW = 6
    MOVE_S = 7
    MOVE_SE = 8
    GRAB = 9
    DROP = 10
