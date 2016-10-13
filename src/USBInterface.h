/*
 * Copyright (c) 2013, Osnabrueck University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the Osnabrueck University nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * USBInterface.h
 *
 *  @date Feb 28, 2013
 *  @author Thomas Wiemann
 */

#ifndef USBINTERFACE_H_
#define USBINTERFACE_H_

#include <libusb.h>
#include <list>
#include <string>
#include <cmath>
#include <cstdint>
#include <unistd.h>

using std::string;
using std::fabs;

enum ChannelMode
{
    Servo=0,
    ServoMultiplied=1,
    Output=2,
    Input = 3,
};

enum HomeMode
{
    Off=0,
    Ignore=1,
    Goto=2
};


/// <summary>
/// The different serial modes the Maestro can be in.  The serial mode
/// determines how the Command Port, TTL Port, the TTL-level UART, and the
/// command processor are connected.
/// </summary>
enum uscSerialMode
{
    ///<summary>On the Command Port, user can send commands and receive responses.
    ///TTL port/UART are connected to make a USB-to-serial adapter.</summary>
    SERIAL_MODE_USB_DUAL_PORT = 0,

    ///<summary>On the Command Port, user can send commands to Maestro and
    /// simultaneously transmit bytes on the UART TX line, and user
    /// can receive bytes from the Maestro and the UART RX line.
    /// TTL port does not do anything.</summary>
    SERIAL_MODE_USB_CHAINED = 1,

    /// <summary>
    /// On the UART, user can send commands and receive reponses after
    /// sending a 0xAA byte to indicate the baud rate.
    /// Command Port receives bytes from the RX line.
    /// TTL Port does not do anything.
    /// </summary>
    SERIAL_MODE_UART_DETECT_BAUD_RATE = 2,

    /// <summary>
    /// On the UART, user can send commands and receive reponses
    /// at a predetermined, fixed baud rate.
    /// Command Port receives bytes from the RX line.
    /// TTL Port does not do anything.
    /// </summary>
    SERIAL_MODE_UART_FIXED_BAUD_RATE = 3,
};


enum uscRequest
{
    REQUEST_GET_PARAMETER = 0x81,
    REQUEST_SET_PARAMETER = 0x82,
    REQUEST_GET_VARIABLES = 0x83,
    REQUEST_SET_SERVO_VARIABLE = 0x84,
    REQUEST_SET_TARGET = 0x85,
    REQUEST_CLEAR_ERRORS = 0x86,
    REQUEST_GET_SERVO_SETTINGS = 0x87,

    // GET STACK and GET CALL STACK are only used on the Mini Maestro.
    REQUEST_GET_STACK = 0x88,
    REQUEST_GET_CALL_STACK = 0x89,
    REQUEST_SET_PWM = 0x8A,

    REQUEST_REINITIALIZE = 0x90,
    REQUEST_ERASE_SCRIPT = 0xA0,
    REQUEST_WRITE_SCRIPT = 0xA1,
    REQUEST_SET_SCRIPT_DONE = 0xA2, // value.low.b is 0 for go, 1 for stop, 2 for single-step
    REQUEST_RESTART_SCRIPT_AT_SUBROUTINE = 0xA3,
    REQUEST_RESTART_SCRIPT_AT_SUBROUTINE_WITH_PARAMETER = 0xA4,
    REQUEST_RESTART_SCRIPT = 0xA5,
    REQUEST_START_BOOTLOADER = 0xFF
};


/// <summary>
/// An object that represents the settings for one servo,
/// e.g. the information in the Settings tab.  One of these objects
/// corresponds to one ServoSettingsControl.
/// </summary>
struct channelSetting
{
    channelSetting()
    {
        name = "";
        mode = ChannelMode::Servo;
        homeMode = HomeMode::Off;
        home = 6000;
        minimum = 3968;
        maximum = 8000;
        neutral = 6000;
        range = 1905;
        speed = 0;
        acceleration = 0;
    }


    /// <summary>
    /// Name.  The Usc class stores this in the registry, not the device.
    /// </summary>
    string name;

    /// <summary>
    /// Type (servo, output, input).
    /// </summary>
    ChannelMode mode;

    /// <summary>
    /// HomeType (off, ignore, goto).
    /// </summary>
    HomeMode homeMode;

    /// <summary>
    /// Home position: the place to go on startup.
    /// If type==servo, units are 0.25 us (qus).
    /// If type==output, the threshold between high and low is 1500.
    ///
    /// This value is only saved on the device if homeType == Goto.
    /// </summary>
    uint16_t home;

    /// <summary>
    /// Minimum (units of 0.25 us, but stored on the device in units of 16 us).
    /// </summary>
    uint16_t minimum;

    /// <summary>
    /// Maximum (units of 0.25 us, but stored on the device in units of 16 us).
    /// </summary>
    uint16_t maximum;

    /// <summary>
    /// Neutral: the center of the 8-bit set target command (value at 127).
    /// If type==servo, units are 0.25 us (qus).
    /// If type==output, the threshold between high and low is 1500.
    /// </summary>
    uint16_t neutral;

    /// <summary>
    /// Range: the +/- extent of the 8-bit command.
    ///   8-bit(254) = neutral + range,
    ///   8-bit(0) = neutral - range
    /// If type==servo units are 0.25 us (qus) (but stored on device in
    /// units of 127*0.25us = 31.75 us.
    /// Range = 0-127*255 = 0-32385 qus.
    /// Increment = 127 qus
    /// </summary>
    uint16_t range;

    /// <summary>
    /// Speed: the maximum change in position (qus) per update.  0 means no limit.
    /// Units depend on your settings.
    /// Stored on device in this format: [0-31]*2^[0-7]
    /// Range = 0-31*2^7 = 0-3968.
    /// Increment = 1.
    ///
    /// Note that the *current speed* is stored on the device in units
    /// of qus, and so it is not subject to the restrictions above!
    /// It can be any value 0-65535.
    /// </summary>
    uint16_t speed;

    /// <summary>
    /// Acceleration: the max change in speed every 80 ms.  0 means no limit.
    /// Units depend on your settings.
    /// Range = 0-255.
    /// Increment = 1.
    /// </summary>
    uint8_t acceleration;
};


struct uscSettings
{
    uscSettings()
    {
        servosAvailable = 6;
        servoPeriod = 156;
        miniMaestroServoPeriod = 80000;
        servoMultiplier = 1;
        serialMode = uscSerialMode::SERIAL_MODE_UART_DETECT_BAUD_RATE;
        fixedBaudRate = 9600;
        enableCrc = false;
        neverSuspend = false;
        serialDeviceNumber = 12;
        miniSscOffset = 0;
        serialTimeout = 0;
        scriptDone = true;
        enablePullups = false;
    }


    /// <summary>
    /// The number of servo ports available (0-5).  This, along with the
    /// servoPeriod, determine the "maximum maximum pulse width".
    /// </summary>
    uint8_t servosAvailable;

    /// <summary>
    /// This setting only applies to the Micro Maestro.
    /// For the Mini Maestro, see miniMaestroServoPeriod.
    ///
    /// The total time allotted to each servo channel, in units of
    /// 256/12 = 21.33333 us.  The unit for this one are unusual, because
    /// that is the way it is stored on the device and its unit is not
    /// a multiple of 4, so we would have inevitable rounding errors if we
    /// tried to represent it in quarter-microseconds.
    ///
    /// Default is 156, so with 6 servos available you get ~20ms between
    /// pulses on a given channel.
    /// </summary>
    uint8_t  servoPeriod;

    /// <summary>
    /// This setting only applies to the Mini Maestro.
    /// For the Micro Maestro, see microMaestroServoPeriod.
    ///
    /// The length of the time period in which the Mini Maestro sends pulses
    /// to all the enabled servos, in units of quarter microseconds.
    ///
    /// Valid values for this parameter are 0 to 16,777,215.  But
    ///
    /// Default is 80000, so each servo receives a pulse every 20 ms (50 Hz).
    /// </summary>
    uint32_t miniMaestroServoPeriod;

    /// <summary>
    /// This setting only applied to the Mini Maestro.
    /// The non-multiplied servos have a period specified by miniMaestroServoPeriod.
    /// The multiplied servos have a period specified by miniMaestroServoPeriod*servoMultiplier.
    ///
    /// Valid values for this parameter are 1 to 256.
    /// </summary>
    uint16_t servoMultiplier;

    /// <summary>
    /// Determines how serial bytes flow between the two USB COM ports, the TTL port,
    /// and the Maestro's serial command processor.
    /// </summary>
    uscSerialMode serialMode;

    /// <summary>
    /// The fixed baud rate, in units of bits per second.  This gets stored in a
    /// different format on the usc.cs, so there will be rounding errors
    /// which get bigger at higher baud rates, but they will be less than
    /// 1% for baud rates of 120000 or less.
    ///
    /// This parameter only applies if serial mode is USB UART Fixed Baud.
    ///
    /// All values above 184 are valid, but values significantly higher than
    /// 250000 are subject to high rounding errors and the usc firmware might not
    /// be able to keep up with those higher data rates.  If the baud rate is too
    /// high and the firmware can't keep up, the Maestro will indicate this to you
    /// by generating a serial overrun or buffer full error.
    /// </summary>
    uint32_t fixedBaudRate;

    /// <summary>
    /// If true, then you must send a 7-bit CRC byte at the end of every serial
    /// command (except the Mini SSC II command).
    /// </summary>
    bool enableCrc;

    /// <summary>
    /// If true, then the Maestro will never go to sleep.  This lets you power
    /// the processer off of USB even when the computer has gone to sleep and put
    /// all of its USB devices in the suspend state.
    /// </summary>
    bool neverSuspend;

    /// <summary>
    /// The serial device number used to identify this device in Pololu protocol
    /// commands.  Valid values are 0-127, default is 12.
    /// </summary>
    uint8_t serialDeviceNumber;

    /// <summary>
    /// The offset used to determine which Mini SSC commands this device will
    /// respond to.  The second byte of the Mini SSC command contains the servo
    /// number; the correspondence between servo number and maestro number (0-5)
    /// is servo# = miniSSCoffset + channel#.  Valid values are 0-254.
    /// </summary>
    uint8_t miniSscOffset;

    /// <summary>
    /// The time it takes for a serial timeout error to occur, in units of 10 ms.
    /// A value of 0 means no timeout error will occur.  All values 0-65535 are valid.
    /// </summary>
    uint16_t serialTimeout;

    /// <summary>
    /// True if the script should not be started when the device starts up.
    /// False if the script should be started.
    /// </summary>
    bool scriptDone;

    /// <summary>
    /// A list of the configurable parameters for each channel, including
    /// name, type, home type, home position, range, neutral, min, max.
    /// </summary>
    std::list<channelSetting> channelSettings;

    /// <summary>
    /// If true, this setting enables pullups for each channel 18-20 which
    /// is configured as an input.  This makes the input value be high by
    /// default, allowing the user to connect a button or switch without
    /// supplying their own pull-up resistor.  Thi setting only applies to
    /// the Mini Maestro 24-Channel Servo Controller.
    /// </summary>
    bool enablePullups;
};

struct servoStatus
{
    /// <summary>The position in units of quarter-microseconds.</summary>
    uint16_t position;

    /// <summary>The target position in units of quarter-microseconds.</summary>
    uint16_t target;

    /// <summary>The speed limit.  Units depends on your settings.</summary>
    uint16_t speed;

    /// <summary>The acceleration limit.  Units depend on your settings.</summary>
    uint8_t acceleration;
};

struct maestroStatus
{
    /// <summary>
    /// The number of values on the data stack (0-32).  A value of 0 means the stack is empty.
    /// </summary>
    uint8_t stackPointer;

    /// <summary>
    /// The number of return locations on the call stack (0-10).  A value of 0 means the stack is empty.
    /// </summary>
    uint8_t callStackPointer;

    /// <summary>
    /// The error register.  Each bit stands for a different error (see uscError).
    /// If the bit is one, then it means that error occurred some time since the last
    /// GET_ERRORS serial command or CLEAR_ERRORS USB command.
    /// </summary>
    uint16_t errors;

    /// <summary>
    /// The address (in bytes) of the next bytecode instruction that will be executed.
    /// </summary>
    uint16_t programCounter;

    /// <summary>Meaningless bytes to protect the program from stack underflows.</summary>
    /// <remarks>This is public to avoid mono warning CS0169.</remarks>
    int16_t buffer[3];

    /// <summary>
    /// The data stack used by the script.  The values in locations 0 through stackPointer-1
    /// are on the stack.
    /// </summary>
    int16_t stack[32];

    /// <summary>
    /// The call stack used by the script.  The addresses in locations 0 through
    /// callStackPointer-1 are on the call stack.  The next return will make the
    /// program counter go to callStack[callStackPointer-1].
    /// </summary>
    uint16_t callStack[10];

    /// <summary>
    /// 0 = script is running.
    /// 1 = script is done.
    /// 2 = script will be done as soon as it executes one more instruction
    ///     (used to implement step-through debugging features)
    /// </summary>
    uint8_t scriptDone;

    /// <summary>Meaningless byte to protect the program from call stack overflows.</summary>
    /// <remarks>This is public to avoid mono warning CS0169.</remarks>
    uint8_t buffer2;
};

/// <summary>
/// The different parameters that can be read or written with REQUEST_GET_PARAMETER
/// and REQUEST_SET_PARAMETER.  These values should only be used by the Usc class.
/// </summary>
enum uscParameter
{
    PARAMETER_INITIALIZED = 0, // 1 byte - 0 or 0xFF
    PARAMETER_SERVOS_AVAILABLE = 1, // 1 byte - 0-5
    PARAMETER_SERVO_PERIOD = 2, // 1 byte - ticks allocated to each servo/256
    PARAMETER_SERIAL_MODE = 3, // 1 byte unsigned value.  Valid values are SERIAL_MODE_*.  Init variable.
    PARAMETER_SERIAL_FIXED_BAUD_RATE = 4, // 2-byte unsigned value; 0 means autodetect.  Init parameter.
    PARAMETER_SERIAL_TIMEOUT = 6, // 2-byte unsigned value
    PARAMETER_SERIAL_ENABLE_CRC = 8, // 1 byte boolean value
    PARAMETER_SERIAL_NEVER_SUSPEND = 9, // 1 byte boolean value
    PARAMETER_SERIAL_DEVICE_NUMBER = 10, // 1 byte unsigned value, 0-127
    PARAMETER_SERIAL_BAUD_DETECT_TYPE = 11, // 1 byte value

    PARAMETER_IO_MASK_C = 16, // 1 byte - pins used for I/O instead of servo
    PARAMETER_OUTPUT_MASK_C = 17, // 1 byte - outputs that are enabled

    PARAMETER_CHANNEL_MODES_0_3                 = 12, // 1 byte - channel modes 0-3
    PARAMETER_CHANNEL_MODES_4_7                 = 13, // 1 byte - channel modes 4-7
    PARAMETER_CHANNEL_MODES_8_11                = 14, // 1 byte - channel modes 8-11
    PARAMETER_CHANNEL_MODES_12_15               = 15, // 1 byte - channel modes 12-15
    PARAMETER_CHANNEL_MODES_16_19               = 16, // 1 byte - channel modes 16-19
    PARAMETER_CHANNEL_MODES_20_23               = 17, // 1 byte - channel modes 20-23
    PARAMETER_MINI_MAESTRO_SERVO_PERIOD_L = 18, // servo period: 3-byte unsigned values, units of quarter microseconds
    PARAMETER_MINI_MAESTRO_SERVO_PERIOD_HU = 19,
    PARAMETER_ENABLE_PULLUPS = 21,  // 1 byte: 0 or 1
    PARAMETER_SCRIPT_CRC = 22, // 2 bytes - stores a checksum of the bytecode program, for comparison
    PARAMETER_SCRIPT_DONE = 24, // 1 byte - copied to scriptDone on startup
    PARAMETER_SERIAL_MINI_SSC_OFFSET = 25, // 1 byte (0-254)
    PARAMETER_SERVO_MULTIPLIER = 26, // 1 byte (0-255)

    // 9 * 24 = 216, so we can safely start at 30
    PARAMETER_SERVO0_HOME = 30, // 2 byte home position (0=off; 1=ignore)
    PARAMETER_SERVO0_MIN = 32, // 1 byte min allowed value (x2^6)
    PARAMETER_SERVO0_MAX = 33, // 1 byte max allowed value (x2^6)
    PARAMETER_SERVO0_NEUTRAL = 34, // 2 byte neutral position
    PARAMETER_SERVO0_RANGE = 36, // 1 byte range
    PARAMETER_SERVO0_SPEED = 37, // 1 byte (5 mantissa,3 exponent) us per 10ms
    PARAMETER_SERVO0_ACCELERATION = 38, // 1 byte (speed changes that much every 10ms)
    PARAMETER_SERVO1_HOME = 39, // 2 byte home position (0=off; 1=ignore)
    PARAMETER_SERVO1_MIN = 41, // 1 byte min allowed value (x2^6)
    PARAMETER_SERVO1_MAX = 42, // 1 byte max allowed value (x2^6)
    PARAMETER_SERVO1_NEUTRAL = 43, // 2 byte neutral position
    PARAMETER_SERVO1_RANGE = 45, // 1 byte range
    PARAMETER_SERVO1_SPEED = 46, // 1 byte (5 mantissa,3 exponent) us per 10ms
    PARAMETER_SERVO1_ACCELERATION = 47, // 1 byte (speed changes that much every 10ms)
    PARAMETER_SERVO2_HOME = 48, // 2 byte home position (0=off; 1=ignore)
    PARAMETER_SERVO2_MIN = 50, // 1 byte min allowed value (x2^6)
    PARAMETER_SERVO2_MAX = 51, // 1 byte max allowed value (x2^6)
    PARAMETER_SERVO2_NEUTRAL = 52, // 2 byte neutral position
    PARAMETER_SERVO2_RANGE = 54, // 1 byte range
    PARAMETER_SERVO2_SPEED = 55, // 1 byte (5 mantissa,3 exponent) us per 10ms
    PARAMETER_SERVO2_ACCELERATION = 56, // 1 byte (speed changes that much every 10ms)
    PARAMETER_SERVO3_HOME = 57, // 2 byte home position (0=off; 1=ignore)
    PARAMETER_SERVO3_MIN = 59, // 1 byte min allowed value (x2^6)
    PARAMETER_SERVO3_MAX = 60, // 1 byte max allowed value (x2^6)
    PARAMETER_SERVO3_NEUTRAL = 61, // 2 byte neutral position
    PARAMETER_SERVO3_RANGE = 63, // 1 byte range
    PARAMETER_SERVO3_SPEED = 64, // 1 byte (5 mantissa,3 exponent) us per 10ms
    PARAMETER_SERVO3_ACCELERATION = 65, // 1 byte (speed changes that much every 10ms)
    PARAMETER_SERVO4_HOME = 66, // 2 byte home position (0=off; 1=ignore)
    PARAMETER_SERVO4_MIN = 68, // 1 byte min allowed value (x2^6)
    PARAMETER_SERVO4_MAX = 69, // 1 byte max allowed value (x2^6)
    PARAMETER_SERVO4_NEUTRAL = 70, // 2 byte neutral position
    PARAMETER_SERVO4_RANGE = 72, // 1 byte range
    PARAMETER_SERVO4_SPEED = 73, // 1 byte (5 mantissa,3 exponent) us per 10ms
    PARAMETER_SERVO4_ACCELERATION = 74, // 1 byte (speed changes that much every 10ms)
    PARAMETER_SERVO5_HOME = 75, // 2 byte home position (0=off; 1=ignore)
    PARAMETER_SERVO5_MIN = 77, // 1 byte min allowed value (x2^6)
    PARAMETER_SERVO5_MAX = 78, // 1 byte max allowed value (x2^6)
    PARAMETER_SERVO5_NEUTRAL = 79, // 2 byte neutral position
    PARAMETER_SERVO5_RANGE = 81, // 1 byte range
    PARAMETER_SERVO5_SPEED = 82, // 1 byte (5 mantissa,3 exponent) us per 10ms
    PARAMETER_SERVO5_ACCELERATION = 83, // 1 byte (speed changes that much every 10ms)
};


const uint8_t servoParameterBytes = 9;


class USBPololuInterface
{
public:
    USBPololuInterface(uint16_t vendorID = 8187, uint16_t productID = 137);
    ~USBPololuInterface();

    void setTarget(int servo, ushort value);
    void setSpeed(int servo, ushort value);
    void moveToTarget(int servo, ushort value);
    void moveToTarget(int servo, ushort value, ushort time);
    void getServoStatus(int servo, servoStatus& status);

    void setUscSettings(uscSettings settings);

    static int MOVE_TARGET_TIMEOUT;
    static int MOVE_TARGET_MAX_TRYS;
private:

    void controlTransfer(char requestType, char request, ushort value, ushort index);
    void controlTransfer(char requestType, char request, ushort Value, ushort Index, unsigned char * data, ushort length);


    uscParameter    specifyServo(uscParameter p, uint8_t servo);
    void            setRawParameter(ushort parameter, ushort value, int bytes);
    uint8_t         channelToPort(uint8_t channel);
    uint8_t         normalSpeedToExponentialSpeed(ushort normalSpeed);

    string getErrorDescription(int code);
    libusb_device*              m_device;
    libusb_context*             m_context;
    libusb_device_handle*       m_deviceHandle;

};


#endif /* USBINTERFACE_H_ */
