from enum import Enum

class CMD(Enum):
    PING = 0
    SEND_TWO_INTS = 1
    SEND_THREE_FLOATS = 2
    ECHO = 3
    DANCE = 4
    SET_VEL = 5
    GET_TIME_MILLIS = 6
    TIME_LOOP = 7
    SEND_TIME_DATA = 8
    GET_TEMP_READINGS = 9
    CALCULATE_DATA_RATE = 10
    COLLECT_IMU_DATA = 11
    SEND_IMU_DATA_ACC = 12
    SEND_IMU_DATA_GYRO = 13
    START_IMU_DATA = 14
    STOP_IMU_DATA = 15
    TIME_OF_FLIGHT = 16
    GATHER_2TOF_AND_IMU_DATA = 17
    CALIBRATION = 18