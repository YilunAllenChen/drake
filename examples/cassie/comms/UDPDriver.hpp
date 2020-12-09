#ifndef CASSIE_UDP_DRIVER
#define CASSIE_UDP_DRIVER

#include <arpa/inet.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <lcm/lcm-cpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

// Cassie LCM messages
#include "cassie_types/lcmt_cassie_command.hpp"
#include "cassie_types/lcmt_cassie_sensor.hpp"
#include "cassie_types/lcmt_cassie_status.hpp"

// Data sizes
#define UDP_HEADER_SIZE 2
#define UTIME_SIZE 8
#define NUMSIZE 4
#define DATA_IN_SIZE (280 + UTIME_SIZE + UDP_HEADER_SIZE)
#define DATA_OUT_SIZE (320 + UTIME_SIZE + UDP_HEADER_SIZE)

#define CASSIE_FREQ 2000
#define STATUS_FREQ 200

class UDPDriver {
 public:
  explicit UDPDriver(boost::shared_ptr<lcm::LCM> &lcm);
  ~UDPDriver();

  boost::shared_ptr<lcm::LCM> lcm_;
  unsigned char dataOut[DATA_OUT_SIZE];

  void publishSensor(unsigned char *buf);
  void publishStatus(unsigned char *buf);

  void commandHandler(const lcm::ReceiveBuffer *rbuf,
                      const std::string &channel,
                      const cassie_types::lcmt_cassie_command *msg);
};

#endif
