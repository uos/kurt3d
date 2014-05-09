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
 * USBInterface.cpp
 *
 *  @date Feb 28, 2013
 *  @author Thomas Wiemann
 */

#include "USBInterface.h"

#include <iostream>
using std::cout;
using std::endl;

int USBPololuInterface::MOVE_TARGET_TIMEOUT = 1000;
int USBPololuInterface::MOVE_TARGET_MAX_TRYS = 1000;

USBPololuInterface::USBPololuInterface(uint16_t v_id, uint16_t p_id)
: m_device(0), m_context(0), m_deviceHandle(0)
{
    // Create new libusb context
    libusb_init(&m_context);

    // Open device
    struct libusb_device **devs;
    struct libusb_device_descriptor info;

    // Get list of devices and counts
    int count = libusb_get_device_list(m_context,&devs);

    // Walk the list, read descriptors, and dump some output from each
    int pololuCount = 0;
    for (int i = 0; i < count;i++)
    {
        libusb_get_device_descriptor(devs[i],&info);

        if(info.idVendor == v_id && info.idProduct == p_id)
        {
            pololuCount++;
            m_device = devs[i];
        }
    }

    // Warn if several boards are connected
    if(pololuCount > 1)
    {
        cout << "USBPololuInterface: Detected " << pololuCount << "boards. Using last detected one." << endl;
    }

    // Try to open device
    int ret;
    if(m_device)
    {
        ret = libusb_open(m_device, &m_deviceHandle);
    }

    if(ret < 0)
    {
        cout << "USBPololuInterface: Unable to open device." << endl;
    }

}



string USBPololuInterface::getErrorDescription(int code)
{
    switch(code)
    {
    case -1:
        return "I/O error.";
    case -2:
        return "Invalid parameter.";
    case -3:
        return "Access denied.";
    case -4:
        return "Device does not exist.";
    case -5:
        return "No such entity.";
    case -6:
        return "Busy.";
    case -7:
        return "Timeout.";
    case -8:
        return "Overflow.";
    case -9:
        return "Pipe error.";
    case -10:
        return "System call was interrupted.";
    case -11:
        return "Out of memory.";
    case -12:
        return "Unsupported/unimplemented operation.";
    case -99:
        return "Other error.";
    default:
        return "Unknown error code.";
    };
}

void USBPololuInterface::controlTransfer(char requestType, char request, ushort value, ushort index)
{
    if(m_deviceHandle)
    {
        int result = libusb_control_transfer(
                m_deviceHandle,
                requestType,
                request,
                value,
                index,
                0, 0, 5000);

        if(result < 0)
        {
            cout << "USBPololuInterface::controlTransfer(): " << getErrorDescription(result) << endl;
        }
    }
    else
    {
        cout << "USBPololuInterface::controlTransfer(): Device not open." << endl;
    }
}

void USBPololuInterface::controlTransfer(
        char requestType,
        char request,
        ushort value,
        ushort index,
        unsigned char* data,
        ushort length)
{
    if(m_deviceHandle)
    {
        int result = libusb_control_transfer(
                m_deviceHandle,
                requestType,
                request,
                value,
                index,
                data, length, 5000);

        if(result < 0)
        {
            cout << "USBPololuInterface::controlTransfer(): " << getErrorDescription(result) << endl;
        }
    }
    else
    {
        cout << "USBPololuInterface::controlTransfer(): Device not open." << endl;
    }
}


uint8_t USBPololuInterface::channelToPort(uint8_t channel)
{
    if (channel <= 3)
    {
        return channel;
    }
    else if (channel < 6)
    {
        return uint8_t(channel + 2);
    }
    cout << "USBPololuInterface::channelToPort(): Unsupported channel: << " << channel << endl;
    return 0;
}


void USBPololuInterface::setRawParameter(ushort parameter, ushort value, int bytes)
{
    ushort index = (ushort)((bytes << 8) + parameter); // high bytes = # of bytes
    controlTransfer(0x40, uscRequest::REQUEST_SET_PARAMETER, value, index);
}


uscParameter USBPololuInterface::specifyServo(uscParameter p, uint8_t servo)
{
    return (uscParameter)((uint8_t)(p) + servo * servoParameterBytes);
}

uint8_t USBPololuInterface::normalSpeedToExponentialSpeed(ushort normalSpeed)
{
    ushort mantissa = normalSpeed;
    uint8_t exponent = 0;

    while (true)
    {
        if (mantissa < 32)
        {
            // We have reached the correct representation.
            return (uint8_t)(exponent + (mantissa << 3));
        }

        if (exponent == 7)
        {
            // The number is too big to express in this format.
            return 0xFF;
        }

        // Try representing the number with a bigger exponent.
        exponent += 1;
        mantissa >>= 1;
    }
}

void USBPololuInterface::setUscSettings(uscSettings settings)
{
    // Serial settings are not supported
    setRawParameter(uscParameter::PARAMETER_SCRIPT_DONE, (ushort)(settings.scriptDone ? 1 : 0), sizeof(ushort));
    setRawParameter(uscParameter::PARAMETER_SERVOS_AVAILABLE, settings.servosAvailable, sizeof(uint8_t));
    setRawParameter(uscParameter::PARAMETER_SERVO_PERIOD, settings.servoPeriod, sizeof(uint8_t));


    uint8_t ioMask = 0;
    uint8_t outputMask = 0;
    uint8_t* channelModeBytes = new uint8_t[6]{0,0,0,0,0,0};

    std::list<channelSetting>::iterator it;

    int i = 0;
    for(it = settings.channelSettings.begin(); it != settings.channelSettings.end(); it++)
    {
        channelSetting setting = *it;


        if (setting.mode == ChannelMode::Input || setting.mode == ChannelMode::Output)
        {
            ioMask |= (uint8_t)(1 << channelToPort(i));
        }

        if (setting.mode == ChannelMode::Output)
        {
            outputMask |= (uint8_t)(1 << channelToPort(i));
        }



        // Make sure that HomeMode is "Ignore" for inputs.  This is also done in
        // fixUscSettings.
        HomeMode correctedHomeMode = setting.homeMode;
        if (setting.mode == ChannelMode::Input)
        {
            correctedHomeMode = HomeMode::Ignore;
        }

        // Compute the raw value of the "home" parameter.
        ushort home;
        if (correctedHomeMode == HomeMode::Off) home = 0;
        else if (correctedHomeMode == HomeMode::Ignore) home = 1;
        else home = setting.home;

        setRawParameter(specifyServo(uscParameter::PARAMETER_SERVO0_HOME, i), home, sizeof(ushort));
        setRawParameter(specifyServo(uscParameter::PARAMETER_SERVO0_MIN, i), (ushort)(setting.minimum / 64), sizeof(ushort));
        setRawParameter(specifyServo(uscParameter::PARAMETER_SERVO0_MAX, i), (ushort)(setting.maximum / 64), sizeof(ushort));
        setRawParameter(specifyServo(uscParameter::PARAMETER_SERVO0_NEUTRAL, i), setting.neutral, sizeof(uint16_t));
        setRawParameter(specifyServo(uscParameter::PARAMETER_SERVO0_RANGE, i), (ushort)(setting.range / 127), sizeof(ushort));
        setRawParameter(specifyServo(uscParameter::PARAMETER_SERVO0_SPEED, i), normalSpeedToExponentialSpeed(setting.speed), sizeof(uint16_t));
        setRawParameter(specifyServo(uscParameter::PARAMETER_SERVO0_ACCELERATION, i), setting.acceleration, sizeof(uint8_t));
    }

    setRawParameter(uscParameter::PARAMETER_IO_MASK_C, ioMask, sizeof(uint8_t));
    setRawParameter(uscParameter::PARAMETER_OUTPUT_MASK_C, outputMask, sizeof(uint8_t));

}





void USBPololuInterface::getServoStatus(int servo, servoStatus& status)
{
    // Alloc memory for maestro board status struct and 6 servo status structs
    int packet_size = sizeof(maestroStatus) + 6 * sizeof(servoStatus);
    unsigned char* buffer = new unsigned char[packet_size];

    // Request data
    controlTransfer(0xC0, REQUEST_GET_VARIABLES, 0, 0, buffer, packet_size);

    // Parse buffer data into servo status
    unsigned char* p = buffer;
    servoStatus servos[6];
    for(int i = 0; i < 6; i++)
    {
        servos[i] = *(servoStatus*)(p + sizeof(maestroStatus) + sizeof(servoStatus) * i);
    }

    // Set output data
    status.position     = servos[servo].position / 4;
    status.target       = servos[servo].target / 4;
    status.speed        = servos[servo].speed;
    status.acceleration = servos[servo].acceleration;
}

void USBPololuInterface::setTarget(int servo, ushort value)
{
    // Target values have to be set in micro seconds. Conversion factor
    // from absolute values to mic. sec. is '4' (see Pololu SDK)
    controlTransfer(0x40, REQUEST_SET_TARGET, 4.0 * value, servo);
}

void USBPololuInterface::moveToTarget(int servo, ushort target)
{
    setTarget(servo, target);
    servoStatus status;

    // Wait until servo has reached target position
    int i = 0;
    do
    {
        usleep(MOVE_TARGET_TIMEOUT);
        getServoStatus(servo, status);
        i++;
    }
    while(status.position != target && i < MOVE_TARGET_MAX_TRYS);

    // Check for timeout
    if(i == MOVE_TARGET_MAX_TRYS)
    {
        cout << "USBPololuInterface::moveToTarget(): Timeout. Target position " << target << " out of servo range?" << endl;
    }
}

void USBPololuInterface::moveToTarget(int servo, ushort target, ushort time)
{
    servoStatus status;
    getServoStatus(servo, status);

    // Calculate steps to target and timeout
    int steps   = std::fabs(status.position - target);
    int timeout = time * 1000 / steps;

    // Decide if we have to increment or decrement
    int initialPosition = status.position;
    int factor = 1;
    if(initialPosition > target) factor *= -1;

    // Move to target
    for(int i = 0; i <= steps; i++)
    {
        setTarget(servo, initialPosition + factor * i);
        usleep(timeout);
    }

}

void USBPololuInterface::setSpeed(int servo, ushort value)
{
    controlTransfer(0x40, REQUEST_SET_SERVO_VARIABLE, value, servo);
}

USBPololuInterface::~USBPololuInterface()
{
    // Close device
    if(m_deviceHandle)
    {
        libusb_close(m_deviceHandle);
    }

    // Release context if valid
    if(m_context)
    {
        libusb_exit(m_context);
    }


}
