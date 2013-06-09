/*
    x_IMU_Arduino_Example.ino
    Author: Seb Madgwick

    Example usage of x-IMU C++ library.  Also uses the quaternion library to
    convert the received quaternion to Euler angles.

    Requires two hardware serial modules: one to receive from the x-IMU and one
    to transmit the decoded data to be displayed on computer.

    x-IMU settings:
    Auxiliary Port > Auxiliary Port Mode: "UART"
    Auxiliary Port > UART > Baud Rate: 115200
    Auxiliary Port > UART > Hardware Flow Control: Off

    Hardware connections:
    x-IMU GND -> Arduino MEGA GND
    x-IMU EXT -> Arduino MEGA 5V
    x-IMU AX2 -> Arduino MEGA RX1

    Tested with "arduino-1.0.3" and "Arduino MEGA".
*/

//------------------------------------------------------------------------------
// Includes

#include "Quaternion.h"
#include "XimuReceiver.h"

//------------------------------------------------------------------------------
// Variables

XimuReceiver ximuReceiver;

//------------------------------------------------------------------------------
// Functions

void setup() {
    Serial.begin(115200);   // for sending data to computer
    Serial1.begin(115200);  // for receiving data from x-IMU
}

void loop() {
    ErrorCode e = ERR_NO_ERROR;

    // Process recieved data
    while(Serial1.available() > 0) {
        e = ximuReceiver.processNewChar(Serial1.read());
    }

    // Print error code (receive error)
    if(e != ERR_NO_ERROR) {
        Serial.print("ERROR: ");
        Serial.print(e);
        Serial.print("\r");
    }

    // Print battery and thermometer data
    if(ximuReceiver.isBattAndThermGetReady()) {
        BattAndThermStruct battAndThermStruct = ximuReceiver.getBattAndTherm();
        Serial.print("battery = ");
        Serial.print(battAndThermStruct.battery);
        Serial.print(", thermometer = ");
        Serial.print(battAndThermStruct.thermometer);
        Serial.print("\r");
    }

    // Print sensor data
    if(ximuReceiver.isInertialAndMagGetReady()) {
        InertialAndMagStruct inertialAndMagStruct = ximuReceiver.getInertialAndMag();
        Serial.print("gyrX = ");
        Serial.print(inertialAndMagStruct.gyrX);
        Serial.print(", gyrY = ");
        Serial.print(inertialAndMagStruct.gyrY);
        Serial.print(", gyrZ = ");
        Serial.print(inertialAndMagStruct.gyrZ);
        Serial.print(", accX = ");
        Serial.print(inertialAndMagStruct.accX);
        Serial.print(", accY = ");
        Serial.print(inertialAndMagStruct.accY);
        Serial.print(", accZ = ");
        Serial.print(inertialAndMagStruct.accZ);
        Serial.print(", magX = ");
        Serial.print(inertialAndMagStruct.magX);
        Serial.print(", magY = ");
        Serial.print(inertialAndMagStruct.magY);
        Serial.print(", magZ = ");
        Serial.print(inertialAndMagStruct.magZ);
        Serial.print("\r");
    }

    // Print quaternion data as Euler angles
    if(ximuReceiver.isQuaternionGetReady()) {
        QuaternionStruct quaternionStruct = ximuReceiver.getQuaternion();
        Quaternion quaternion = Quaternion(quaternionStruct.w, quaternionStruct.x, quaternionStruct.y, quaternionStruct.z);
        EulerAnglesStruct eulerAnglesStruct = quaternion.getEulerAngles();
        Serial.print("roll = ");
        Serial.print(eulerAnglesStruct.roll);
        Serial.print(", pitch = ");
        Serial.print(eulerAnglesStruct.pitch);
        Serial.print(", yaw = ");
        Serial.print(eulerAnglesStruct.yaw);
        Serial.print("\r");
    }
}

//------------------------------------------------------------------------------
// End of file
