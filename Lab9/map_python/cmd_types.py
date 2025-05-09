from enum import Enum

class CMD(Enum):
    START_RECORD_DATA = 0
    STOP_RECORD_DATA = 1
    GET_DATA_RATE = 2
    PID_POSITION_CONTROL = 3
    PID_ORIEN_CONTROL = 4
    TURN_LEFT = 5
    TURN_RIGHT = 6
    STOP = 7
    KF_DATA = 8
    FLIP = 9
    MAP = 10
    LOCALIZATION = 11
    NAVIGATION = 12