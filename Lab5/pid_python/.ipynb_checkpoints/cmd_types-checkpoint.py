from enum import Enum

class CMD(Enum):
    START_RECORD_DATA = 0
    STOP_RECORD_DATA = 1
    GET_DATA_RATE = 2
    PID_POSITION_CONTROL = 3
    STOP = 4