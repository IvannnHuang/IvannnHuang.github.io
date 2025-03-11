from enum import Enum

class CMD(Enum):
    PING = 0
    SEND_TWO_INTS = 1
    SEND_THREE_FLOATS = 2
    ECHO = 3
    DANCE = 4
    SET_VEL = 5
    GET_TIME_MILLIS = 6
    GET_TIME_MILLIS_1s = 7
    TIME_DATA_ARRAY = 8
    SEND_TIME_DATA = 9
    TIME_TEMP_DATA_ARRAY = 10
    GET_TEMP_READINGS = 11