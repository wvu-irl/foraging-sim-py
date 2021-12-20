from enum import Enum, unique

@unique
class Actions(Enum):
    STAY = 0
    MOVE_E = 1
    MOVE_NE = 2
    MOVE_N = 3
    MOVE_NW = 4
    MOVE_W = 5
    MOVE_SW = 6
    MOVE_S = 7
    MOVE_SE = 8
    PIVOT_CCW = 9
    PIVOT_CW = 10
    GRAB = 11
    DROP = 12
