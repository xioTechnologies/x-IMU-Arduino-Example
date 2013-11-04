/*
    XimuReceiver.cpp
    Author: Seb Madgwick

    C++ library for receiving data from the x-IMU.  Current implementation
    supports only a few packet types.

    See x_IMU_Arduino_Example.ino for example usage.
*/

//------------------------------------------------------------------------------
// Includes

#include "XimuReceiver.h"

//------------------------------------------------------------------------------
// Definitions

enum PacketHeaders {
    PKT_ERROR,
    PKT_COMMAND,
    PKT_READ_REGISTER,
    PKT_WRITE_REGISTER,
    PKT_READ_DATE_TIME,
    PKT_WRITE_DATE_TIME,
    PKT_RAW_BATTERY_AND_THERMOMETER_DATA,
    PKT_CAL_BATTERY_AND_THERMOMETER_DATA,
    PKT_RAW_INERTIAL_AND_MAGNETIC_DATA,
    PKT_CAL_INERTIAL_AND_MAGNETIC_DATA,
    PKT_QUATERNION_DATA,
    PKT_DIGITAL_IO_DATA,
    PKT_RAW_ANALOGUE_INPUT_DATA,
    PKT_CAL_ANALOGUE_INPUT_DATA,
    PKT_PWM_OUTPUT_DATA,
    PKT_RAW_DXL345_BUS_DATA,
    PKT_CAL_DXL345_BUS_DATA,
};

enum PacketLengths {
    LEN_ERROR = 4,
    LEN_COMMAND = 6,
    LEN_READ_REGISTER = 4,
    LEN_WRITE_REGISTER = 8,
    LEN_READ_DATE_TIME = 2,
    LEN_WRITE_DATE_TIME = 8,
    LEN_RAW_BATTERY_AND_THERMOMETER_DATA = 6,
    LEN_CAL_BATTERY_AND_THERMOMETER_DATA = 6,
    LEN_RAW_INERTIAL_AND_MAGNETIC_DATA = 20,
    LEN_CAL_INERTIAL_AND_MAGNETIC_DATA = 20,
    LEN_QUATERNION_DATA = 10,
    LEN_DIGITAL_IO_DATA = 4,
    LEN_RAW_ANALOGUE_INPUT_DATA = 18,
    LEN_CAL_ANALOGUE_INPUT_DATA = 18,
    LEN_PWM_OUTPUT_DATA = 10,
    LEN_RAW_DXL345_BUS_DATA = 26,
    LEN_CAL_DXL345_BUS_DATA = 26,
};


typedef enum {
    Q_CALIBRATED_BATTERY = 12,
    Q_CALIBRATED_THERMOEMTER = 8,
    Q_CALIBRATED_GYROSCOPE = 4,
    Q_CALIBRATED_ACCELEROEMETER = 11,
    Q_CALIBRATED_MAGNETOMETER = 11,
    Q_QUATERNION = 15,
    Q_BATTERY_SENSITIVITY = 5,
    Q_BATTERY_BIAS = 8,
    Q_THERMOMETER_SENSITIVITY = 6,
    Q_THERMOMETER_BIAS = 0,
    Q_GYROSCOPE_SENSITIVITY = 7,
    Q_GYROSCOPE_SAMPLED_200DPS = 0,
    Q_GYROSCOPE_BIAS_AT_25DEGC = 3,
    Q_GYROSCOPE_BIAS_TEMP_SENSITIVITY = 11,
    Q_GYROSCOPE_SAMPLED_BIAS = 3,
    Q_ACCELEROMETER_SENSITIVITY = 4,
    Q_ACCELEROMETER_BIAS = 8,
    Q_ACCELEROMETER_Sampled1g = 4,
    Q_MAGNETOMETER_SENSITIVITY = 4,
    Q_MAGNETOMETER_BIAS = 8,
    Q_MAGNETOMETER_HARD_IRON_BIAS = 11,
    Q_ALGORITHM_KP = 11,
    Q_ALGORITHM_KI = 15,
    Q_ALGORITHM_INIT_KP = 11,
    Q_ALGORITHM_INIT_PERIOD = 11,
    Q_CALIBRATED_ANALOGUE_INPUT = 12,
    Q_ANALOGUE_INPUT_SENSITIVITY = 4,
    Q_ANALOGUE_INPUT_BIAS = 8,
    Q_CALIRBATED_ADXL345 = 10,
    Q_ADXL345_BUS_SENSITIVITY = 6,
    Q_ADXL345_BUS_BIAS = 8,
} FixedQ;

//------------------------------------------------------------------------------
// Variables

// Serial stream decoding
unsigned char buf[256];
unsigned char bufIndex = 0;

// Decoded data
BattAndThermStruct battAndThermStruct = { 0.0f, 0.0f };
InertialAndMagStruct inertialAndMagStruct = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
QuaternionStruct quaternionStruct = { 1.0f, 0.0f, 0.0f, 0.0f };

// Data ready flags
bool battAndThermGetReady = false;
bool inertialAndMagGetReady = false;
bool quaternionGetReady = false;

//------------------------------------------------------------------------------
// Methods

XimuReceiver::XimuReceiver() {
}

ErrorCode XimuReceiver::processNewChar(unsigned char c) {

    // Add new byte to buffer
    buf[bufIndex++] = c;

    // Process receive buffer if framing char received
    if(c & 0x80) {

        // Calculate packet size
        int packetSize = bufIndex - 1 - ((bufIndex - 1) >> 3);
        bufIndex = 0;   //reset index

        // Extract packet (truncate to discard all msb)
        unsigned char packet[256];
        packet[0]  = (buf[0 ] << 1) | (buf[1 ] >> 6);
        packet[1]  = (buf[1 ] << 2) | (buf[2 ] >> 5);
        packet[2]  = (buf[2 ] << 3) | (buf[3 ] >> 4);
        packet[3]  = (buf[3 ] << 4) | (buf[4 ] >> 3);
        packet[4]  = (buf[4 ] << 5) | (buf[5 ] >> 2);
        packet[5]  = (buf[5 ] << 6) | (buf[6 ] >> 1);
        packet[6]  = (buf[6 ] << 7) | (buf[7 ] >> 0);
        packet[7]  = (buf[8 ] << 1) | (buf[9 ] >> 6);
        packet[8]  = (buf[9 ] << 2) | (buf[10] >> 5);
        packet[9]  = (buf[10] << 3) | (buf[11] >> 4);
        packet[10] = (buf[11] << 4) | (buf[12] >> 3);
        packet[11] = (buf[12] << 5) | (buf[13] >> 2);
        packet[12] = (buf[13] << 6) | (buf[14] >> 1);
        packet[13] = (buf[14] << 7) | (buf[15] >> 0);
        packet[14] = (buf[16] << 1) | (buf[17] >> 6);
        packet[15] = (buf[17] << 2) | (buf[18] >> 5);
        packet[16] = (buf[18] << 3) | (buf[19] >> 4);
        packet[17] = (buf[19] << 4) | (buf[20] >> 3);
        packet[18] = (buf[20] << 5) | (buf[21] >> 2);
        packet[19] = (buf[21] << 6) | (buf[22] >> 1);
        packet[20] = (buf[22] << 7) | (buf[23] >> 0);
        packet[21] = (buf[24] << 1) | (buf[25] >> 6);
        packet[22] = (buf[25] << 2) | (buf[26] >> 5);
        packet[23] = (buf[26] << 3) | (buf[27] >> 4);
        packet[24] = (buf[27] << 4) | (buf[28] >> 3);
        packet[25] = (buf[28] << 5) | (buf[29] >> 2);
        packet[26] = (buf[29] << 6) | (buf[30] >> 1);
        packet[27] = (buf[30] << 7) | (buf[31] >> 0);

        /*
            TODO: Verify checksum here, if fail then return ERR_INVALID_CHECKSUM;
        */

        // Interpret packet according to header
        switch(packet[0]) {

            case PKT_CAL_BATTERY_AND_THERMOMETER_DATA:
                if(packetSize != LEN_CAL_BATTERY_AND_THERMOMETER_DATA) {
                    return ERR_INVALID_NUM_BYTES_FOR_PACKET_HEADER;
                }
                battAndThermStruct.battery = fixedToFloat(concat(packet[1], packet[2]), Q_CALIBRATED_BATTERY);
                battAndThermStruct.thermometer = fixedToFloat(concat(packet[3], packet[4]), Q_CALIBRATED_THERMOEMTER);
                battAndThermGetReady = true;
                break;

            case(PKT_CAL_INERTIAL_AND_MAGNETIC_DATA):
                if(packetSize != LEN_CAL_INERTIAL_AND_MAGNETIC_DATA) {
                    return ERR_INVALID_NUM_BYTES_FOR_PACKET_HEADER;
                }
                inertialAndMagStruct.gyrX = fixedToFloat(concat(packet[1], packet[2]), Q_CALIBRATED_GYROSCOPE);
                inertialAndMagStruct.gyrY = fixedToFloat(concat(packet[3], packet[4]), Q_CALIBRATED_GYROSCOPE);
                inertialAndMagStruct.gyrZ = fixedToFloat(concat(packet[5], packet[6]), Q_CALIBRATED_GYROSCOPE);
                inertialAndMagStruct.accX = fixedToFloat(concat(packet[7], packet[8]), Q_CALIBRATED_ACCELEROEMETER);
                inertialAndMagStruct.accY = fixedToFloat(concat(packet[9], packet[10]), Q_CALIBRATED_ACCELEROEMETER);
                inertialAndMagStruct.accZ = fixedToFloat(concat(packet[11], packet[12]), Q_CALIBRATED_ACCELEROEMETER);
                inertialAndMagStruct.magX = fixedToFloat(concat(packet[13], packet[14]), Q_CALIBRATED_MAGNETOMETER);
                inertialAndMagStruct.magY = fixedToFloat(concat(packet[15], packet[16]), Q_CALIBRATED_MAGNETOMETER);
                inertialAndMagStruct.magZ = fixedToFloat(concat(packet[17], packet[18]), Q_CALIBRATED_MAGNETOMETER);
                inertialAndMagGetReady = true;
                break;

            case(PKT_QUATERNION_DATA):
                if(packetSize != LEN_QUATERNION_DATA) {
                    return ERR_INVALID_NUM_BYTES_FOR_PACKET_HEADER;
                }
                quaternionStruct.w = fixedToFloat(concat(packet[1], packet[2]), Q_QUATERNION);
                quaternionStruct.x = fixedToFloat(concat(packet[3], packet[4]), Q_QUATERNION);
                quaternionStruct.y = fixedToFloat(concat(packet[5], packet[6]), Q_QUATERNION);
                quaternionStruct.z = fixedToFloat(concat(packet[7], packet[8]), Q_QUATERNION);
                quaternionGetReady = true;
                break;

            default:
                break;
        }
    }
    return ERR_NO_ERROR;
}

float XimuReceiver::fixedToFloat(const short fixed, const unsigned char q) const {
    return (float)fixed / (float)(1 << q);
}

unsigned short XimuReceiver::concat(const unsigned char msb, const unsigned char lsb) const {
    return ((unsigned short)msb << 8) | (unsigned short)lsb;
}

bool XimuReceiver::isBattAndThermGetReady(void) const {
    return battAndThermGetReady;
}

bool XimuReceiver::isInertialAndMagGetReady(void) const {
    return inertialAndMagGetReady;
}

bool XimuReceiver::isQuaternionGetReady(void) const {
    return quaternionGetReady;
}

BattAndThermStruct XimuReceiver::getBattAndTherm(void) {
    battAndThermGetReady = false;
    return battAndThermStruct;
}

InertialAndMagStruct XimuReceiver::getInertialAndMag(void) {
    inertialAndMagGetReady = false;
    return inertialAndMagStruct;
}

QuaternionStruct XimuReceiver::getQuaternion(void) {
    quaternionGetReady = false;
    return quaternionStruct;
}

//------------------------------------------------------------------------------
// End of file