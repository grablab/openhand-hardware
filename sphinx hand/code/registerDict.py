#Yale GrabLab 2020
#Added XL series

#This document serves to provide addresses for the RX/MX, X* Dynamixel Series (Protocol1 and Protocol 2), and XL-320
#These addresses will be used in the lib_robotis_mod file in order to read and write values to the dynamixel device.


RXMX_Series = {
 "ADDR_MODEL_NUMBER_L"              : 0,        # 1 bytes (R)
 "ADDR_MODEL_NUMBER_H"              : 1,        # 1 bytes (R)
 "ADDR_VERSION_OF_FIRMWARE"         : 2,        # 1 byte (R)
 "ADDR_ID"                          : 3,        # 1 bytes (RW)
 "ADDR_BAUD_RATE"                   : 4,        # 1 byte (RW)
 "ADDR_RETURN_DELAY_TIME"           : 5,        # 1 byte (RW)
 "ADDR_CW_ANGLE_LIMIT_L"            : 6,        # 1 byte (RW)
 "ADDR_CW_ANGLE_LIMIT_H"            : 7,        # 1 byte (RW)
 "ADDR_CCW_ANGLE_LIMIT_L"           : 8,        # 1 byte (RW)
 "ADDR_CCW_ANGLE_LIMIT_H"           : 9,        # 1 byte (RW)
 "ADDR_HIGHEST_LIMIT_TEMP"          : 11,       # 1 byte (RW)
 "ADDR_LOWEST_LIMIT_VOLTAGE"        : 12,       # 1 bytes (RW)
 "ADDR_HIGHEST_LIMIT_VOLTAGE"       : 13,       # 1 bytes (RW)
 "ADDR_MAX_TORQUE_L"                : 14,       # 1 byte (RW)
 "ADDR_MAX_TORQUE_H"                : 15,       # 1 bytes (RW)
 "ADDR_STATUS_RETURN_LEVEL"         : 16,       # 1 bytes (RW)
 "ADDR_ALARM_LED"                   : 17,       # 1 bytes (RW)
 "ADDR_ALARM_SHUTDOWN"              : 18,       # 1 bytes (RW)
 "ADDR_TORQUE_ENABLE"               : 24,       # 1 bytes (RW)
 "ADDR_LED"                         : 25,       # 1 bytes (RW)
 "ADDR_CW_COMPLIANCE_MARGIN"        : 26,       # 1 bytes (RW)
 "ADDR_CCW_COMPLIANCE_MARGIN"       : 27,       # 1 bytes (RW)
 "ADDR_CW_COMPLIANCE_SLOPE"         : 28,       # 1 byte (RW)
 "ADDR_CCW_COMPLIANCE_SLOPE"        : 29,       # 1 byte (RW)
 "ADDR_GOAL_POSITION_L"             : 30,       # 1 byte (RW)
 "ADDR_GOAL_POSITION_H"             : 31,       # 1 byte (RW)
 "ADDR_MOVING_SPEED_L"              : 32,       # 1 byte (R)
 "ADDR_MOVING_SPEED_H"              : 33,       # 1 byte (R)
 "ADDR_TORQUE_LIMIT_L"              : 34,       # 1 bytes (RW)
 "ADDR_TORQUE_LIMIT_H"              : 35,       # 1 bytes (RW)
 "ADDR_PRESENT_POSITION_L"          : 36,       # 1 bytes (RW)
 "ADDR_PRESENT_POSITION_H"          : 37,       # 1 bytes (RW)
 "ADDR_PRESENT_SPEED_L"             : 38,       # 1 bytes (RW)
 "ADDR_PRESENT_SPEED_H"             : 39,       # 1 bytes (RW)
 "ADDR_PRESENT_LOAD_L"              : 40,       # 1 bytes (RW)
 "ADDR_PRESENT_LOAD_H"              : 41,       # 1 byte (RW)
 "ADDR_PRESENT_VOLTAGE"             : 42,       # 1 bytes (RW)
 "ADDR_PRESENT_TEMPERATURE"         : 43,       # 1 bytes (RW)
 "ADDR_REGISTERED"                  : 44,       # 1 bytes (RW)
 "ADDR_MOVING"                      : 46,       # 1 bytes (RW)
 "ADDR_LOCK"                        : 47,       # 1 bytes (RW)
 "ADDR_PUNCH_L"                     : 48,       # 1 bytes (RW)
 "ADDR_PUNCH_H"                     : 49,       # 1 bytes (RW)
 "ADDR_CURRENT_L"                   : 68,       # 1 bytes (R)
 "ADDR_TORQUE_CONTROL_MODE_ENABLE"  : 70,       # 1 byte (RW)
 "ADDR_GOAL_TORQUE_L"               : 71,       #1 byte (RW)


 "TORQUE_ENABLE"                            : 1,                   # Value for enabling the torque
 "TORQUE_DISABLE"                           : 0,                   # Value for disabling the torque
 "CURRENT_CONTROL_MODE"                     : 0,
 "VELOCITY_CONTROL_MODE"                    : 1,
 "POSITION_CONTROL_MODE"                    : 3,
 "EXTENDED_POSITION_CONTROL_MODE"           : 4,
 "CURRENT_BASED_POSITION_CONTROL_MODE"      : 5,
 "PWM_CONTROL_MODE"                         : 16

} #End RXMX-Series Dictionary


X_Series = {
 "ADDR_MODEL_NUMBER"                : 0,        # 2 bytes (R)
 "ADDR_MODEL_INFORMATION"           : 2,        # 4 bytes (R)
 "ADDR_VERSION_OF_FIRMWARE"         : 6,        # 1 byte (R)
 "ADDR_ID"                          : 7,        # 1 bytes (RW)
 "ADDR_BAUD_RATE"                   : 8,        # 1 byte (RW)
 "ADDR_RETURN_DELAY_TIME"           : 9,        # 1 byte (RW)
 "ADDR_DRIVE_MODE"                  : 10,       # 1 byte (RW)   (direction of rotationm, takes 0 or 1)
 "ADDR_OPERATING_MODE"              : 11,       # 1 byte (RW)
 "ADDR_SECONDARY_ID"                : 12,       # 1 byte (RW) (for grouping dynamixels)
 "ADDR_PROTOCOL_VERSION"            : 13,       # 1 byte (RW)
 "ADDR_HOMING_OFFSET"               : 20,       # 4 bytes (RW)
 "ADDR_MOVING_THRESHOLD"            : 24,       # 4 bytes (RW)
 "ADDR_TEMPERATURE_LIMIT"           : 31,       # 1 byte (RW)
 "ADDR_MAX_VOLTAGE_LIMIT"           : 32,       # 2 bytes (RW)
 "ADDR_MIN_VOLTAGE_LIMIT"           : 34,       # 2 bytes (RW)
 "ADDR_PWM_LIMIT"                   : 36,       # 2 bytes (RW)
 "ADDR_CURRENT_LIMIT"               : 38,       # 2 bytes (RW)
 "ADDR_ACCELERATION_LIMIT"          : 40,       # 4 bytes (RW)
 "ADDR_VELOCITY_LIMIT"              : 44,       # 4 bytes (RW)
 "ADDR_MAX_POSITION_LIMIT"          : 48,       # 4 bytes (RW)
 "ADDR_MIX_POSITION_LIMIT"          : 52,       # 4 bytes (RW)
 "ADDR_SHUTDOWN"                    : 63,       # 1 byte (RW)
 "ADDR_TORQUE_ENABLE"               : 64,       # 1 byte (RW)
 "ADDR_LED"                         : 65,       # 1 byte (RW)
 "ADDR_STATUS_RETURN_LEVEL"         : 68,       # 1 byte (RW)
 "ADDR_REGISTERED_INSTRUCTION"      : 69,       # 1 byte (R)
 "ADDR_HARDWARE_ERROR_STATUS"       : 70,       # 1 byte (R)
 "ADDR_VELOCITY_I_GAIN"             : 76,       # 2 bytes (RW)
 "ADDR_VELOCITY_P_GAIN"             : 78,       # 2 bytes (RW)
 "ADDR_POSITION_D_GAIN"             : 80,       # 2 bytes (RW)
 "ADDR_POSITION_I_GAIN"             : 82,       # 2 bytes (RW)
 "ADDR_POSITION_P_GAIN"             : 84,       # 2 bytes (RW)
 "ADDR_FEEDFORWARD_2ND_GAIN"        : 88,       # 2 bytes (RW)
 "ADDR_FEEDFORWARD_1ST_GAIN"        : 90,       # 2 bytes (RW)
 "ADDR_BUS_WATCHDOG"                : 98,       # 1 byte (RW)
 "ADDR_GOAL_PWM"                    : 100,      # 2 bytes (RW)
 "ADDR_GOAL_CURRENT"                : 102,      # 2 bytes (RW)
 "ADDR_GOAL_VELOCITY"               : 104,      # 4 bytes (RW)
 "ADDR_PROFILE_ACCELERATION"        : 108,      # 4 bytes (RW)
 "ADDR_PROFILE_VELOCITY"            : 112,      # 4 bytes (RW)
 "ADDR_GOAL_POSITION"               : 116,      # 4 bytes (RW)
 "ADDR_REALTIME_TICK"               : 120,      # 2 bytes (R)
 "ADDR_MOVING"                      : 122,      # 1 byte (R)
 "ADDR_MOVING_STATUS"               : 123,      # 1 byte (R)
 "ADDR_PRESENT_PWM"                 : 124,      # 2 bytes (RW)
 "ADDR_PRESENT_CURRENT"             : 126,      # 2 bytes (R)
 "ADDR_PRESENT_VELOCITY"            : 128,      # 4 bytes (R)
 "ADDR_PRESENT_POSITION"            : 132,      # 4 bytes (R)
 "ADDR_VELOCITY_TRAJECTORY"         : 136,      # 4 bytes (R)
 "ADDR_POSITION_TRAJECTORY"         : 140,      # 4 bytes (R)
 "ADDR_PRESENT_INPUT_VOLTAGE"       : 144,      # 2 bytes (R)
 "ADDR_PRESENT_TEMPERATURE"         : 146,      # 1 byte (R)

 "TORQUE_ENABLE"                            : 1,                   # Value for enabling the torque
 "TORQUE_DISABLE"                           : 0,                   # Value for disabling the torque
 "CURRENT_CONTROL_MODE"                     : 0,
 "VELOCITY_CONTROL_MODE"                    : 1,
 "POSITION_CONTROL_MODE"                    : 3,
 "EXTENDED_POSITION_CONTROL_MODE"           : 4,
 "CURRENT_BASED_POSITION_CONTROL_MODE"      : 5,
 "PWM_CONTROL_MODE"                         : 16

} #End X-Series Dictionary

XL_Series = {
 "ADDR_MODEL_NUMBER"                : 0,        # 2 bytes (R)
 "ADDR_VERSION_OF_FIRMWARE"         : 2,        # 1 byte (R)
 "ADDR_ID"                          : 3,        # 1 bytes (RW)
 "ADDR_BAUD_RATE"                   : 4,        # 1 byte (RW)
 "ADDR_RETURN_DELAY_TIME"           : 5,        # 1 byte (RW)
 "ADDR_CW_ANGLE_LIMIT"              : 6,        # 2 bytes (RW)
 "ADDR_CCW_ANGLE_LIMIT"             : 8,        # 2 bytes (RW)
 "ADDR_CONTROL_MODE"                : 11,       # 1 byte (RW)
 "ADDR_TEMPERATURE_LIMIT"           : 12,       # 1 byte (RW)
 "ADDR_MIN_VOLTAGE_LIMIT"           : 13,       # 1 byte (RW)
 "ADDR_MAX_VOLTAGE_LIMIT"           : 14,       # 1 byte (RW)
 "ADDR_MAX_TORQUE"                  : 15,       # 2 bytes (RW)
 "ADDR_STATUS_RETURN_LEVEL"         : 17,       # 1 bytes (RW)
 "ADDR_SHUTDOWN"                    : 18,       # 1 byte (RW)
 "ADDR_TORQUE_ENABLE"               : 24,       # 1 byte (RW)
 "ADDR_LED"                         : 25,       # 1 byte (RW)
 "ADDR_D_GAIN"                      : 27,       # 1 bytes (RW)
 "ADDR_I_GAIN"                      : 28,       # 1 bytes (RW)
 "ADDR_P_GAIN"                      : 29,       # 1 bytes (RW)
 "ADDR_GOAL_POSITION"               : 30,       # 2 bytes (RW)
 "ADDR_MOVING_SPEED"                : 32,       # 2 bytes (RW)
 "ADDR_TORQUE_LIMIT"                : 35,       # 2 bytes (RW)
 "ADDR_PRESENT_POSITION"            : 37,       # 2 bytes (R)
 "ADDR_PRESENT_SPEED"               : 39,       # 2 bytes (R)
 "ADDR_PRESENT_LOAD"                : 41,       # 2 bytes (R) 
 "ADDR_PRESENT_VOLTAGE"             : 45,       # 1 bytes (R)
 "ADDR_PRESENT_TEMPERATURE"         : 46,       # 1 byte (R)
 "ADDR_REGISTERED"                  : 47,       # 1 byte (R)
 "ADDR_MOVING"                      : 49,       # 1 byte (R)
 "ADDR_HARDWARE_ERROR_STATUS"       : 50,       # 1 byte (R)
 "ADDR_PUNCH"                       : 51,       # 2 bytes (RW)

} #End XL-Series Dictionary


# Macro for Control Table Value
def DXL_MAKEWORD(a, b):
    return (a & 0xFF) | ((b & 0xFF) << 8)

def DXL_MAKEDWORD(a, b):
    return (a & 0xFFFF) | (b & 0xFFFF) << 16

def DXL_LOWORD(l):
    return l & 0xFFFF

def DXL_HIWORD(l):
    return (l >> 16) & 0xFFFF

def DXL_LOBYTE(w):
    return w & 0xFF

def DXL_HIBYTE(w):
    return (w >> 8) & 0xFF

def updateCRC(crc_accum, data_blk_ptr, data_blk_size):
    crc_table = [0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202]

    for j in range(0, data_blk_size):
        i = ((crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF
        crc_accum = ((crc_accum << 8) ^ crc_table[i]) & 0xFFFF

    return crc_accum