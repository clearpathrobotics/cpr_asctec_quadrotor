#include "cpr_asctec_driver/asctec_serial.h"
#include <iostream>
#include <cstring>
#include <boost/assign/list_of.hpp>
#include <boost/unordered_map.hpp>

namespace cpr_asctec_driver
{

  // TODO(pbovbel): Implement more packet types as necessary:
  // http://wiki.asctec.de/display/AR/Communicating+with+the+Low+Level+Processor

  const boost::unordered_map<AsctecSerial::DataType, const uint16_t> AsctecSerial::packet_request = boost::assign::map_list_of
      (STATUS, 0x0001)
      (IMU_RAW, 0x0002)
      (IMU_CALC, 0x0004)
      (GPS, 0x0080);

  const boost::unordered_map<AsctecSerial::DataType, const uint8_t> AsctecSerial::packet_desc = boost::assign::map_list_of
      (IMU_RAW, 0x01)
      (STATUS, 0x02)
      (IMU_CALC, 0x03)
      (GPS, 0x23);

  const boost::unordered_map<AsctecSerial::DataType, const std::size_t> AsctecSerial::packet_size = boost::assign::map_list_of
      (IMU_RAW, sizeof(IMU_RAWDATA))
      (STATUS, sizeof(LL_STATUS))
      (IMU_CALC, sizeof(IMU_CALCDATA))
      (GPS, sizeof(GPS_DATA));

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
    return true;
  }

  bool AsctecSerial::requestData(std::set<AsctecSerial::DataType> requests)
  {
    POLL_REQUEST request;

    for (std::set<DataType>::iterator it = requests.begin(); it != requests.end(); ++it)
    {
      request.packets += packet_request.at(*it);
    }

    std::size_t wrote = serial_.write((uint8_t *) &request, sizeof(request));
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
        }
        else if (header.packet_desc == packet_desc.at(IMU_CALC))
        {
          type = IMU_CALC;
          imu_calc = *(IMU_CALCDATA *) (response.c_str() + i);
        }
        else if (header.packet_desc == packet_desc.at(IMU_RAW))
        {
          type = IMU_RAW;
          imu_raw = *(IMU_RAWDATA *) (response.c_str() + i);
        }
        else if (header.packet_desc == packet_desc.at(GPS))
        {
          type = GPS;
          gps = *(GPS_DATA *) (response.c_str() + i);
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
