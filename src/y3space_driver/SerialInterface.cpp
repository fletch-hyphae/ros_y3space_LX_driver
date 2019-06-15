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

#include <y3space_driver/SerialInterface.h>

SerialInterface::SerialInterface(ros::NodeHandle& nh):
	m_logger("[ Y3SSerialInterface ] ")
{
    nh.param<int>("BAUD_RATE", m_baudrate, 115200);	//115200 is default for 3Space devices
    nh.param<std::string>("SERIAL_PORT", m_port, "/dev/imu");
}

SerialInterface::~SerialInterface()
{
    if (connection != NULL)
    {
      	if(connection->isOpen())
      	{
        	ROS_INFO_STREAM(this->logger << " Closing the Serial Port");
        	connection->close();
      	}
    }
}

void SerialInterface::serialConnect()
{
	try
	{
  		connection.reset(new Serial(m_port, (uint32_t)m_baud, Timeout::simpleTimeout(60000)));
	}
	catch(IOException &e)
	{
  		std::string ioerror = e.what();
  		ROS_ERROR_STREAM(this->logger << "Unable to connect port: " << m_port.c_str());
  		ROS_ERROR_STREAM(this->logger << "Is the serial port open?\nError: "  << ioerror.c_str());
	}

	if(connection && connection->isOpen())
	{
  		ROS_INFO_STREAM(this->logger
                  		<< "Connection Established with Port: " << m_port.c_str()
                  		<< " with baudrate: " << m_baud);
	}
}

void SerialInterface::serialWrite(uint8_t *buf, size_t len)
{
    size_t written = this->connection->write(buf, len);
    if (written != len)
    {
    	ROS_WARN_STREAM(this->logger << "Len: " << len  << "; Written: " << written);
    }
}

void SerialInterface::serialWriteString(const std::string& str)
{
    size_t written = this->connection->write(str);
}

uint8_t SerialInterface::serialReadByte()
{
    uint8_t buf;
    if(this->connection->available() > 0)
    {
      	size_t bytes = this->connection->read(&buf, 1);
      	if (bytes != 1)
      	{
        	ROS_WARN_STREAM(this->logger << "Unable to read");
      	}
    }
    return buf;
}

std::string serialReadLine()
{
    std::string str = this->connection->readline();
    return str;
}

uint8_t* serialReadBytes(size_t nbytes)
{
    uint8_t *buf = (uint8_t *)malloc(sizeof(uint8_t) * nbytes);
    if(this->connection->available() > 0)
    {
      	size_t bytes = this->connection->read(buf, nbytes);
      	if (bytes != nbytes)
      	{
        	ROS_WARN_STREAM(this->logger << "Unable to read");
      	}
    }
    return buffer;
}

virtual inline size_t available()
{
    return this->connection->available();
}
