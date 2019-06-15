#include <y3space_driver/Y3SpaceDriver.h>


Y3SpaceDriver::Y3SpaceDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh):
    SerialInterface(pnh),
    m_pnh(pnh),
    m_nh(nh)
{
    this->serialConnect();
    this->m_imuPub = this->yostlab_nh_.advertise<sensor_msgs::Imu>("/imu/filtered", 10);
    this->m_tempPub = this->m_nh.advertise<std_msgs::Float64>("/imu/temp", 10);
}

Y3SpaceDriver::~Y3SpaceDriver() {}

void Y3SpaceDriver::restoreFactorySettings()
{
    this->serialWriteString(RESTORE_FACTORY_SETTINGS);
}

const std::string Y3SpaceDriver::getSoftwareVersion()
{
    this->serialWriteString(GET_FIRMWARE_VERSION_STRING);

    const std::string buf = this->serialReadLine();
    ROS_INFO_STREAM(this->logger << "Software version: " << buf);
    return buf;
}

const std::string Y3SpaceDriver::getAxisDirection()
{
    this->serialWriteString(GET_AXIS_DIRECTION);

    const std::string buf = this->serialReadLine();
    const std::string ret = [&]()
    {
        if(buf == "0\r\n")
        {
            return "X: Right, Y: Up, Z: Forward";
        }
        else if ( buf == "1\r\n")
        {
            return "X: Right, Y: Forward, Z: Up";
        }
        else if ( buf == "2\r\n")
        {
            return "X: Up, Y: Right, Z: Forward";
        }
        else if (buf == "3\r\n")
        {
            return "X: Forward, Y: Right, Z: Up";
        }
        else if( buf == "4\r\n")
        {
            return "X: Up, Y: Forward, Z: Right";
        }
        else if( buf == "5\r\n")
        {
            return "X: Forward, Y: Up, Z: Right";
        }
        else if (buf == "19\r\n")
        {
            return "X: Forward, Y:Left, Z: Up";
        }
        else
        {
            return ("Unknown. Buffer indicates: " + buf + ")");
        }
    }();

    ROS_INFO_STREAM(this->logger << "Axis Direction: " << ret);
    return ret;
}

void Y3SpaceDriver::startGyroCalibration(void)
{
    ROS_INFO_STREAM(this->logger << "Starting Auto Gyro Calibration...");
    this->serialWriteString(BEGIN_GYRO_AUTO_CALIB);
  
    ros::Duration(5.0).sleep();
    ROS_INFO_STREAM(this->logger << "Proceeding");
}

void Y3SpaceDriver::setMIMode(bool on)
{
    if(on)
    {
        this->serialWriteString(SET_MI_MODE_ENABLED);
    }
    else
    {
        this->serialWriteString(SET_MI_MODE_DISABLED);
    }
}

const std::string Y3SpaceDriver::getCalibMode()
{
    this->serialWriteString(GET_CALIB_MODE);

    const std::string buf = this->serialReadLine();
    const std::string ret = [&]()
    {
        if(buf == "0\r\n")
        {
            return "Bias";
        }
        else if ( buf == "1\r\n")
        {
            return "Scale and Bias";
        }
        else
        {
            return ("Unknown. Buffer indicates: " + buf + ")");
        }
    }();

    ROS_INFO_STREAM(this->logger << "Calibration Mode: " << ret);
    return ret;
}

const std::string Y3SpaceDriver::getMIMode()
{
    this->serialWriteString(GET_MI_MODE_ENABLED);

    const std::string buf = this->serialReadLine();
    const std::string ret = [&]()
    {
        if(buf == "0\r\n")
        {
            return "Disabled";
        }
        else if ( buf == "1\r\n")
        {
            return "Enabled";
        }
        else
        {
            return ("Unknown. Buffer indicates: " + ")");
        }
    }();

    ROS_INFO_STREAM(this->logger << "MI Mode: " << ret_buf << ", buf is: " << buf);
    return ret_buf;
}


//! Run the serial sync
void Y3SpaceDriver::run()
{
    std::vector<double> parsedVals;
    sensor_msgs::Imu imuMsg;
    std_msgs::Float64 tempMsg;

    this->startGyroCalibration();
    this->getSoftwareVersion();
    this->getAxisDirection();
    this->getCalibMode();
    this->getMIMode();
    this->serialWriteString(SET_STREAMING_SLOTS_ROS_IMU_ABSOLUTE);
    this->serialWriteString(TARE_WITH_CURRENT_ORIENTATION);
    this->serialWriteString(TARE_WITH_CURRENT_QUATERNION);
    this->serialWriteString(SET_STREAMING_TIMING_100_MS);
    this->serialWriteString(START_STREAMING);
  
    ros::Rate rate(10);
    int line = 0;
    while(ros::ok())
    {
        while(this->available() > 0)
        {
            line += 1;
            std::string buf = this->serialReadLine();
            std::string parse;
            std::stringstream ss(buf);
            double i;

            // Parse data from the line
            while (ss >> i)
            {
                parsedVals.push_back(i);
                if (ss.peek() == ',')
                ss.ignore();
            }

            // Should stop reading when line == number of tracked streams
            if(line == 4)
            {
                // Reset line tracker
                line = 0;
        
                // Prepare IMU message
                imuMsg.header.stamp           = ros::Time::now();
                imuMsg.header.frame_id        = "imu_link";
                imuMsg.orientation.x          = parsed_val_[0];
                imuMsg.orientation.y          = parsed_val_[1];
                imuMsg.orientation.z          = parsed_val_[2];
                imuMsg.orientation.w          = parsed_val_[3];
                imuMsg.angular_velocity.x     = parsed_val_[4];
                imuMsg.angular_velocity.y     = parsed_val_[5];
                imuMsg.angular_velocity.z     = parsed_val_[6];
                imuMsg.linear_acceleration.x  = parsed_val_[7];
                imuMsg.linear_acceleration.y  = parsed_val_[8];
                imuMsg.linear_acceleration.z  = parsed_val_[9];

                // Prepare temperature message        
                tempMsg.data = parsedVals[10];

                // Clear parsed values
                parsedVals.clear();

                this->imuPub.publish(imuMsg);
                this->tempPub.publish(tempMsg);
            }
        }

        // Throttle ROS at fixed Rate
        rate.sleep();
        ros::spinOnce();
    }
}
