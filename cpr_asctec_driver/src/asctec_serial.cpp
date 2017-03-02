#include "cpr_asctec_driver/asctec_serial.h"
#include <iostream>
#include <cstring>
#include <boost/assign/list_of.hpp>
#include <boost/unordered_map.hpp>

namespace cpr_asctec_driver
{

  const boost::unordered_map<AsctecSerial::DataType, const uint16_t> AsctecSerial::packet_request = boost::assign::map_list_of
      (STATUS, 0x0001)
      (IMU_RAW, 0x0002)
      (IMU_CALC, 0x0004);

  const boost::unordered_map<AsctecSerial::DataType, const uint8_t> AsctecSerial::packet_desc = boost::assign::map_list_of
      (IMU_RAW, 0x01)
      (STATUS, 0x02)
      (IMU_CALC, 0x03);

  const boost::unordered_map<AsctecSerial::DataType, const std::size_t> AsctecSerial::packet_size = boost::assign::map_list_of
      (IMU_RAW, sizeof(IMU_RAWDATA))
      (STATUS, sizeof(LL_STATUS))
      (IMU_CALC, sizeof(IMU_CALCDATA));

  //example CRC16 function
  unsigned short crc_update(unsigned short crc, unsigned char data)
  {
    data ^= (crc & 0xff);
    data ^= data << 4;
    return ((((unsigned short) data << 8) | ((crc >> 8) & 0xff)) ^ (unsigned char) (data >> 4)
            ^ ((unsigned short) data << 3));
  }

  unsigned short crc16(void *data, unsigned short cnt)
  {
    unsigned short crc = 0xff;
    unsigned char *ptr = (unsigned char *) data;
    int i;

    for (i = 0; i < cnt; i++)
    {
      crc = crc_update(crc, *ptr);
      ptr++;
    }
    return crc;
  }

  AsctecSerial::AsctecSerial(std::string port, unsigned int baud)
  {
    serial_.setPort(port);
    serial_.setBaudrate(baud);
    serial::Timeout timeout(serial::Timeout::simpleTimeout(200));
    serial_.setTimeout(timeout);
    serial_.open();
  }

  bool AsctecSerial::sendCommand(const CTRL_INPUT &command)
  {
    std::size_t wrote = serial_.write((uint8_t *) &command, sizeof(command));

    ROS_DEBUG_STREAM("Wrote command " << wrote << " of " << sizeof(command));
    return true;
  }

  bool AsctecSerial::requestData(std::set<AsctecSerial::DataType> requests)
  {
    POLL_REQUEST request;

    int i = 0;
    for (std::set<DataType>::iterator it = requests.begin(); it != requests.end(); ++it)
    {
      i++;
      request.packets += packet_request.at(*it);
    }

    std::size_t wrote = serial_.write((uint8_t *) &request, sizeof(request));
    ROS_DEBUG_STREAM("Wrote " << wrote << " expected " << sizeof(request));


    return true;
  }


  AsctecSerial::DataType AsctecSerial::poll()
  {
    AsctecSerial::DataType type = NONE;
    POLL_HEADER header;
    POLL_FOOTER footer;

    std::string response = serial_.readline(1024, "<#<");

    // find header
    for(std::size_t i = 0; i < response.size(); i++){

      if(strncmp((char *) response.c_str() + i, header.string, sizeof(header.string)) == 0){
        header = *(POLL_HEADER *) (response.c_str() + i);
        i += sizeof(POLL_HEADER);

        //TODO check crc
        footer = *(POLL_FOOTER *) (response.c_str() + i + header.length);

        if (header.packet_desc == packet_desc.at(STATUS))
        {
          type = STATUS;
          status = *(LL_STATUS *) (response.c_str() + i);
          ROS_DEBUG_STREAM("uptime " << status.up_time);
          ROS_DEBUG_STREAM("running " << status.flying);
          ROS_DEBUG_STREAM("on " << status.motors_on);
        }
        else if (header.packet_desc == packet_desc.at(IMU_CALC))
        {
          type = IMU_CALC;
          IMU_CALCDATA imu_calc = *(IMU_CALCDATA *) (response.c_str() + i);
          ROS_DEBUG_STREAM("imu calc " << imu_calc.acc_z);
        }
        else if (header.packet_desc == packet_desc.at(IMU_RAW))
        {
          type = IMU_RAW;
          IMU_RAWDATA imu_raw = *(IMU_RAWDATA *) (response.c_str() + i);
          ROS_DEBUG_STREAM("imu raw " << imu_raw.acc_z);
        }
        else
        {
          type = UNKNOWN;
        }

        break;
      }

    }
    return type;
  }
}