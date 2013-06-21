/*
    XimuReceiver.h
    Author: Seb Madgwick

    C++ library for receiving data from the x-IMU.  Current implementation
    supports only a few packet types.

    See x_IMU_Arduino_Example.ino for example usage.
*/

#ifndef XimuReceiver_h
#define XimuReceiver_h

//------------------------------------------------------------------------------
// Definitions

typedef enum {
    ERR_NO_ERROR,
    ERR_FACTORY_RESET_FAILED,
    ERR_LOW_BATTERY,
    ERR_USB_RECEIVE_BUFFER_OVERRUN,
    ERR_USB_TRANSMIT_BUFFER_OVERRUN,
    ERR_BLUETOOTH_RECEIVE_BUFFER_OVERRUN,
    ERR_BLUETOOTH_TRANSMIT_BUFFER_OVERRUN,
    ERR_SD_CARD_WRITE_BUFFER_OVERRUN,
    ERR_TOO_FEW_BYTES_IN_PACKET,
    ERR_TOO_MANY_BYTES_IN_PACKET,
    ERR_INVALID_CHECKSUM,
    ERR_UNKNOWN_HEADER,
    ERR_INVALID_NUM_BYTES_FOR_PACKET_HEADER,
    ERR_INVALID_REGISTER_ADDRESS,
    ERR_REGISTER_READ_ONLY,
    ERR_INVALID_REGSITER_VALUE,
    ERR_INVALID_COMMAND,
    ERR_GYROSCOPE_AXIS_NOT_AT_200DPS,
    ERR_GYROSCOPE_NOT_STATIONARY,
    ERR_ACCELEROMETER_AXIS_NOT_AT_1G,
    ERR_MMAGNETOMETER_SATURATION,
    ERR_INCORRECT_AUXILIARY_PORT_MODE,
    ERR_UART_RECEIVE_BUFFER_OVERRUN,
    ERR_UART_TRANSMIT_BUFFER_OVERRUN,
} ErrorCode;

typedef struct {
    float battery;      /* battery voltage in V */
    float thermometer;  /* thermometer in degC  */
} BattAndThermStruct;

typedef struct {
    float gyrX;     /* gyroscope x axis in dps    */
    float gyrY;     /* gyroscope y axis in dps    */
    float gyrZ;     /* gyroscope z axis in dps    */
    float accX;     /* accelerometer x axis in g  */
    float accY;     /* accelerometer y axis in g  */
    float accZ;     /* accelerometer z axis in g  */
    float magX;     /* magnetometer x axis in Ga  */
    float magY;     /* magnetometer y axis in Ga  */
    float magZ;     /* magnetometer z axis in Ga  */
} InertialAndMagStruct;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} QuaternionStruct;

//------------------------------------------------------------------------------
// Class declaration

class XimuReceiver {
    public:
        XimuReceiver(void);
        ErrorCode processNewChar(unsigned char c);
        bool isBattAndThermGetReady(void) const;
        bool isInertialAndMagGetReady(void) const;
        bool isQuaternionGetReady(void) const;
        BattAndThermStruct getBattAndTherm(void);
        InertialAndMagStruct getInertialAndMag(void);
        QuaternionStruct getQuaternion(void);

    private:
        unsigned char buf[256];
        unsigned char bufIndex;
        BattAndThermStruct battAndThermStruct;
        InertialAndMagStruct inertialAndMagStruct;
        QuaternionStruct quaternionStruct;
        bool battAndThermGetReady;
        bool inertialAndMagGetReady;
        bool quaternionGetReady;
        float fixedToFloat(const short fixed, const unsigned char q) const;
        unsigned short concat(const unsigned char msb, const unsigned char lsb) const;
};

#endif

//------------------------------------------------------------------------------
// End of file
