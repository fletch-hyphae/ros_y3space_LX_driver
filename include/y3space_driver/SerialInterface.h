/*
 * MIT License
 * 
 * Copyright (c) 2018 Cagatay Sarı
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _Y3Space_SERIAL_INTERFACE_H
#define _Y3Space_SERIAL_INTERFACE_H

#include <malloc.h>
#include <ros/ros.h>
#include <serial/serial.h>

using namespace serial;


class SerialInterface
{

private:
    using SerialPtr = std::unique_ptr<serial::Serial>;

public:
    //!
    //! Constructor
    //!  
    SerialInterface(ros::NodeHandle& nh);
    //!
    //! Destructor
    //!
    virtual ~SerialInterface();
    //!
    //! Establish serial connection
    //!
    virtual void serialConnect();
    //!
    //! Write to byte serial
    //!
    virtual inline void serialWrite(uint8_t *buf, size_t len);
    //!
    //!  Write string to serial
    //!
    virtual inline void serialWriteString(const std::string& str);
    //!
    //! Read byte from serial
    //!
    virtual inline uint8_t serialReadByte();
    //!
    //! Read line from serial
    //!
    virtual inline std::string serialReadLine();
    //!
    //! Read variable number of bytes from serial
    //!
    virtual inline uint8_t* serialReadBytes(size_t nbytes);
    //!
    //! Check if port is available and has bytes
    //!
    virtual inline size_t available();
    //!
    //! Getter for baudrate
    //!
    const int& getBaudRate() { return m_baud; }
    //!
    //! Getter for serial port
    //!
    const std::string& getSerialPort() { return m_port; }

private:
    //!
    //! baudrate
    //!
    int m_baudrate;
    //!
    //! connection related variables
    //!
    SerialPtr m_connection;
    //!
    //! name of the devices port
    //!
    std::string m_port;
    //!
    //! logger zone
    //!
    const std::string m_logger;
};
#endif //_Y3Space_SERIAL_INTERFACE_H
