#ifndef __PARAMS_HPP__
#define __PARAMS_HPP__

#include <string>

static const std::string NODE_NAME      = "vrobot_imu";
static const std::string TOPIC_NAME     = "/imu";
static const std::string SERIAL_PORT    = "/dev/imu";
static const int         SERIAL_BAUD    = 9600;
static const int         SERIAL_BITS    = 8;
static const int         SERIAL_PARITY  = 0;
static const int         SERIAL_STOP    = 1;
static const int         SERIAL_TIMEOUT = 1000;

static const int PUBLISH_RATE = 100;

enum UPADTE {
  ACC_UPDATE   = 0x01,
  GYR_UPDATE   = 0x02,
  ANGLE_UPDATE = 0x03,
  MAG_UPDATE   = 0x04,
  PRESS_UPDATE = 0x05,
};

#endif // __PARAMS_HPP__