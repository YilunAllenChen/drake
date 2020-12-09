#include "UDPDriver.hpp"

// UDP addressing
#define UDP_IP_IN "10.10.10.100"
#define UDP_PORT_IN 25001
#define UDP_IP_OUT "10.10.10.3"
#define UDP_PORT_OUT 25000
// #define UDP_IP_OUT "127.0.0.1"
// #define UDP_PORT_OUT 5005

/////////////////// THREADS ////////////////////////////////
void publishLCMThread(UDPDriver &driver) {
  printf("Started publishLCMThread\n");
  int sock;
  struct sockaddr_in inAddr;
  unsigned char buf[DATA_IN_SIZE];
  int cycles = CASSIE_FREQ / STATUS_FREQ;

  memset(&inAddr, 0, sizeof(inAddr));
  inAddr.sin_family = AF_INET;
  inAddr.sin_port = htons(UDP_PORT_IN);
  inet_pton(AF_INET, UDP_IP_IN, &inAddr.sin_addr.s_addr);

  sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    perror("Cannot create socket");
  }

  int success = bind(sock, (struct sockaddr *)&inAddr, sizeof(inAddr));
  if (success < 0) {
    perror("Bind failed");
  }

  int ii = cycles;
  while (true) {
    // int recvlen = recvfrom(sock, buf, DATA_IN_SIZE, 0, NULL, 0);
    recvfrom(sock, buf, DATA_IN_SIZE, 0, NULL, 0);
    ii++;
    driver.publishSensor(buf);

    if (ii >= cycles) {
      ii = 0;
      driver.publishStatus(buf);
    }
  }

  close(sock);
}

void receiveLCMThread(UDPDriver &driver) {
  printf("Started receiveLCMThread\n");
  int sock;
  struct sockaddr_in outAddr;

  memset(&outAddr, 0, sizeof(outAddr));
  outAddr.sin_family = AF_INET;
  outAddr.sin_port = htons(UDP_PORT_OUT);
  inet_pton(AF_INET, UDP_IP_OUT, &outAddr.sin_addr.s_addr);

  sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    perror("Cannot create socket");
  }

  // double ii = 0;
  while (true) {
    lcm_handle(driver.lcm_->getUnderlyingLCM());
    // int bytesSent = sendto(sock, driver.dataOut, DATA_OUT_SIZE, 0, (struct
    // sockaddr*)&outAddr, sizeof(outAddr));
    sendto(sock, driver.dataOut, DATA_OUT_SIZE, 0, (struct sockaddr *)&outAddr,
           sizeof(outAddr));
  }

  close(sock);
}

/////////////////// MAIN ///////////////////////////////////
int main() {
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if (!lcm->good()) {
    perror("ERROR: lcm is not good");
    return -1;
  }

  UDPDriver *driver = new UDPDriver(lcm);

  // Subscribe to Cassie commands
  lcm->subscribe("CASSIE_COMMAND", &UDPDriver::commandHandler, driver);

  // publishLCMThread(*driver);
  // receiveLCMThread(*driver);

  boost::thread_group thread_group;
  thread_group.create_thread(
      boost::bind(publishLCMThread, boost::ref(*driver)));
  thread_group.create_thread(
      boost::bind(receiveLCMThread, boost::ref(*driver)));
  thread_group.join_all();

  exit(0);
}
