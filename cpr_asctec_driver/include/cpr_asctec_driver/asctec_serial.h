#ifndef PROJECT_ASCTEC_SERIAL_H
#define PROJECT_ASCTEC_SERIAL_H

#include <stdint.h>
#include <serial/serial.h>
#include <ros/ros.h>
#include "boost/unordered_map.hpp"

namespace cpr_asctec_driver
{

/*
 * Available data structures
 */

typedef struct _POLL_REQUEST
{
  char string[4];  // always initialize with ">*>p"
  unsigned short packets;

  _POLL_REQUEST() : packets(0)
  {
    char tmp[] = ">*>p";
    std::copy(tmp, tmp + 4, string);
  };
}__attribute__((packed)) POLL_REQUEST;

typedef struct _POLL_HEADER
{
  char string[3];
  unsigned short length;
  unsigned char packet_desc;

  _POLL_HEADER()
  {
    char tmp[] = ">*>";
    std::copy(tmp, tmp + 3, string);
  };
}__attribute__((packed)) POLL_HEADER;

typedef struct _POLL_FOOTER
{
  unsigned short crc16;
  char string[3];

  _POLL_FOOTER()
  {
    char tmp[] = "<#<";
    std::copy(tmp, tmp + 3, string);
  };
}__attribute__((packed)) POLL_FOOTER;

typedef struct _WP_COMMAND
{
  const char string[4];  // always initialize with ">*>w"
  const char cmd;      // s, g, l, e or h
}__attribute__((packed)) WP_COMMAND;

typedef struct _WP_HEADER
{
  char startstring[2];  // should be ">a"
  unsigned char packet_desc;
}__attribute__((packed)) WP_HEADER;

typedef struct _WP_FOOTER
{
  char stopstring[2];  // should be "a<"
}__attribute__((packed)) WP_FOOTER;

typedef struct _LL_STATUS
{
  //battery voltages in mV
  short battery_voltage_1;
  short battery_voltage_2;
  //don't care
  short status;
  //Controller cycles per second (should be ?1000)
  short cpu_load;
  //don't care
  char compass_enabled;
  char chksum_error;
  char flying;
  char motors_on;
  short flightMode;
  //Time motors are turning
  short up_time;
}__attribute__((packed)) LL_STATUS;

typedef struct _IMU_RAWDATA
{
  //pressure sensor 24-bit value, not scaled but bias free
  int pressure;
  //16-bit gyro readings; 32768 = 2.5V
  short gyro_x;
  short gyro_y;
  short gyro_z;
  //10-bit magnetic field sensor readings
  short mag_x;
  short mag_y;
  short mag_z;
  //16-bit accelerometer readings
  short acc_x;
  short acc_y;
  short acc_z;
  //16-bit temperature measurement using yaw-gyro internal sensor
  unsigned short temp_gyro;
  //16-bit temperature measurement using ADC internal sensor
  unsigned int temp_ADC;
}__attribute__((packed)) IMU_RAWDATA;

typedef struct _IMU_CALCDATA
{
  //angles derived by integration of gyro_outputs, drift compensated by data & !fusion; -90000..+90000 pitch(nick) and roll, 0..360000 yaw; 1000 = 1 & ! degree
  int angle_nick;
  int angle_roll;
  int angle_yaw;
  //angular velocities, raw values 16 bit but bias free
  int angvel_nick;
  int angvel_roll;
  int angvel_yaw;
  //acc-sensor outputs, calibrated: -10000..+10000 = -1g..+1g
  short acc_x_calib;
  short acc_y_calib;
  short acc_z_calib;
  //horizontal / vertical accelerations: -10000..+10000 = -1g..+1g
  short acc_x;
  short acc_y;
  short acc_z;
  //reference angles derived by accelerations only: -90000..+90000; 1000 = 1 & !degree
  int acc_angle_nick;
  int acc_angle_roll;
  //total acceleration measured (10000 = 1g)
  int acc_absolute_value;
  //magnetic field sensors output, offset free and scaled; units not & !determined, as only the direction of the field vector is taken into & ! account
  int Hx;
  int Hy;
  int Hz;
  //compass reading: angle reference for angle_yaw: 0..360000; 1000 = 1 degree
  int mag_heading;
  //pseudo speed measurements: integrated accelerations, pulled towards zero; & !units unknown; used for short-term position stabilization
  int speed_x;
  int speed_y;
  int speed_z;
  //height in mm (after data fusion)
  int height;
  //diff. height in mm/s (after data fusion)
  int dheight;
  //diff. height measured by the pressure sensor mm/s
  int dheight_reference;
  //height measured by the pressure sensor mm
  int height_reference;
}__attribute__((packed)) IMU_CALCDATA;

typedef struct _GPS_DATA
{
  //latitude/longitude in deg * 10^7
  int latitude;
  int longitude;
  //GPS height in mm
  int height;
  //speed in x (E/W) and y(N/S) in mm/s
  int speed_x;
  int speed_y;
  //GPS heading in deg * 1000
  int heading;
  //accuracy estimates in mm and mm/s
  unsigned int horizontal_accuracy;
  unsigned int vertical_accuracy;
  unsigned int speed_accuracy;
  //number of satellite vehicles used in NAV solution
  unsigned int numSV;
  // GPS status information; 0x03 = valid GPS fix
  int status;
}__attribute__((packed)) GPS_DATA;

typedef struct _GPS_DATA_ADVANCED
{
  //latitude/longitude in deg * 10^7
  int latitude;
  int longitude;
  //GPS height in mm
  int height;
  //speed in x (E/W) and y(N/S) in mm/s
  int speed_x;
  int speed_y;
  //GPS heading in deg * 1000
  int heading;
  //accuracy estimates in mm and mm/s
  unsigned int horizontal_accuracy;
  unsigned int vertical_accuracy;
  unsigned int speed_accuracy;
  //number of satellite vehicles used in NAV solution
  unsigned int numSV;
  //GPS status information; 0x03 = valid GPS fix
  int status;
  //coordinates of current origin in deg * 10^7
  int latitude_best_estimate;
  int longitude_best_estimate;
  //velocities in X (E/W) and Y (N/S) after data fusion
  int speed_x_best_estimate;
  int speed_y_best_estimate;
}__attribute__((packed)) GPS_DATA_ADVANCED;

typedef struct _RC_DATA
{
  //channels as read from R/C receiver
  unsigned short channels_in8;
  //channels bias free, remapped and scaled to 0..4095
  unsigned short channels_out8;
  //Indicator for valid R/C receiption
  unsigned char lock;
}__attribute__((packed)) RC_DATA;

typedef struct _CONTROLLER_OUTPUT
{
  //attitude controller outputs; 0..200 = -100 ..+100%
  int nick;
  int roll;
  int yaw;
  //current thrust (height controller output); 0..200 = 0..100%
  int thrust;
}__attribute__((packed)) CONTROLLER_OUTPUT;

typedef struct _CTRL_INPUT
{

  char header[5];
  short pitch; //pitch input: -2047..+2047 (0=neutral)
  short roll; //roll input: -2047..+2047 (0=neutral)
  short yaw; //(=R/C Stick input) -2047..+2047 (0=neutral)
  short thrust; //collective: 0..4095 = 0..100%
  short ctrl;
  /*control byte:
    bit 0: pitch control enabled
    bit 1: roll control enabled
    bit 2: yaw control enabled
    bit 3: thrust control enabled
    bit 4: height control enabled
    bit 5: GPS position control enabled
    */
  short chksum;

  _CTRL_INPUT(short _roll, short _pitch, short _yaw, short _thrust)
      : pitch(_pitch), roll(_roll), yaw(_yaw), thrust(_thrust),
        ctrl(0b1111), chksum(roll + pitch + yaw + thrust + ctrl - 21846)
  {
    // TODO cleanup
//      char tmp[] = ">*>di";
//      std::copy(tmp, tmp+5, header);
    header[0] = '>';
    header[1] = '*';
    header[2] = '>';
    header[3] = 'd';
    header[4] = 'i';
  }
}__attribute__((packed)) CTRL_INPUT;

typedef struct _WAYPOINT
{
  //always set to 1
  unsigned char wp_number;

  //don't care
  unsigned char dummy_1;
  unsigned short dummy_2;

  //see WPPROP defines below
  unsigned char properties;

  //max. speed to travel to waypoint in % (default 100)
  unsigned char max_speed;

  //time to stay at a waypoint (XYZ) in 1/100th s
  unsigned short time;

  //position accuracy to consider a waypoint reached in mm (default: 2500 (= 2.5 m))
  unsigned short pos_acc;

  //chksum = 0xAAAA + wp.yaw + wp.height + wp.time + wp.X + wp.Y + wp.max_speed +	wp.pos_acc + wp.properties + wp.wp_number;
  short chksum;

  //waypoint coordinates in mm	// longitude in abs coords
  int X;
  //waypoint coordinates in mm	// latitude in abs coords
  int Y;

  //Desired heading at waypoint
  int yaw;

  //height over 0 reference in mm
  int height;
}__attribute__((packed)) WAYPOINT;


class AsctecSerial
{

public:
  typedef boost::shared_ptr<AsctecSerial> Ptr;

  enum DataType
  {
    NONE,
    STATUS,
    IMU_RAW,
    IMU_CALC,
    GPS,
    UNKNOWN
  };

  AsctecSerial(std::string port, unsigned int baud);

  bool requestData(std::set<DataType> requests);

  bool sendCommand(const CTRL_INPUT &command);

  DataType poll();

  LL_STATUS status;
  IMU_CALCDATA imu_calc;
  IMU_RAWDATA imu_raw;
  GPS_DATA gps;

private:

  serial::Serial serial_;

  static const boost::unordered_map<DataType, const uint16_t> packet_request;
  static const boost::unordered_map<DataType, const uint8_t> packet_desc;
  static const boost::unordered_map<DataType, const std::size_t> packet_size;

};

}

#endif //PROJECT_ASCTEC_SERIAL_H
