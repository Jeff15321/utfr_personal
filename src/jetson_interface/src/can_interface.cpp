/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: can_interface.cpp
* auth: Youssef Elhadad
* desc: CAN Interfacing Library for Jetson
*/

#include <can_interface.hpp>

namespace utfr_dv {
namespace jetson_interface {

std::map<int, canid_t> msg_array_dvjet_sensor{
    {(int)msg_dvjet_sensor_e::RBP, 0x008},
    {(int)msg_dvjet_sensor_e::FBP, 0x010},
    {(int)msg_dvjet_sensor_e::SPEEDFL, 0x012},
    {(int)msg_dvjet_sensor_e::SPEEDRL, 0x013},
    {(int)msg_dvjet_sensor_e::FBPT, 0x01B},
    {(int)msg_dvjet_sensor_e::ImuY, 0x174},
    {(int)msg_dvjet_sensor_e::ImuX, 0x178},
    {(int)msg_dvjet_sensor_e::ImuZ, 0x17C},
    {(int)msg_dvjet_sensor_e::DV_THR_COMMAND, 0x0C5},
    {(int)msg_dvjet_sensor_e::STR_RATE_CMD, 0x2B5},
    {(int)msg_dvjet_sensor_e::BRK_RATE_CMD, 0x3C3},
    {(int)msg_dvjet_sensor_e::RES, 0x011},
    {(int)msg_dvjet_sensor_e::ANGSENREC, 0x2B0},
    {(int)msg_dvjet_sensor_e::ANGSENTRA, 0x7C0},
    {(int)msg_dvjet_sensor_e::ANGSENTRA, 0x0A5},
    {(int)msg_dvjet_sensor_e::DVDrivingDynamics1, 0x500},
    {(int)msg_dvjet_sensor_e::DVDrivingDynamics2, 0x501},
    {(int)msg_dvjet_sensor_e::DVSystemStatus, 0x502}};

bool CanInterface::connect(const char *canline) {

  int _sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  if (_sock < 0) {
    // socket did not work
    perror("CAN't connect");
    close(_sock);
    return false;
  }

  sock = _sock;

  strcpy(ifr.ifr_name, canline);

  if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
    perror("IOCTL ERROR");
    close(sock);
    return false;
  }

  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex; // set to 0!

  if (bind(sock, (sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("BIND ERROR");
    close(sock);
    return false;
  }

  // All checks and binds completed!
  return true;
}

int CanInterface::get_can(msg_dvjet_sensor_e msgName) {
  // while(pthread_mutex_trylock(&readlock)){;}
  int result;
  if (msgName == msg_dvjet_sensor_e::ANGSENREC) {
    result = (int)((messages[msg_array_dvjet_sensor[(int)msgName]].data[2]) |
                   (((messages[msg_array_dvjet_sensor[(int)msgName]].data[3])
                     << 8)));
  } else if (msgName == msg_dvjet_sensor_e::MOTPOS) {
    result = (int)((messages[msg_array_dvjet_sensor[(int)msgName]].data[0]) |
                   (((messages[msg_array_dvjet_sensor[(int)msgName]].data[1])
                     << 8)));
  } else {
    result = ARRAY_TO_INT64(
        messages[msg_array_dvjet_sensor[(int)msgName]]
            .data); // messages[msg_array_dvjet_sensor[(int)msgName]].data;
  }

  // pthread_mutex_unlock(&readlock);
  return result;
}

static void *thread_read(void *node) {
  utfr_dv::jetson_interface::CanInterface *canNode =
      (utfr_dv::jetson_interface::CanInterface *)node;
  pthread_mutex_lock(&(canNode->lock));
  while (1) {
    struct canfd_frame recieved;

    while (pthread_mutex_trylock(&(canNode->readlock))) {
      ;
    }
    read(canNode->sock, &recieved, sizeof(struct canfd_frame));
    pthread_mutex_unlock(&(canNode->readlock));
    canNode->messages[recieved.can_id] = recieved;
  }
  pthread_mutex_unlock(&(canNode->lock));
  return NULL;
}

int CanInterface::read_can() {
  if (pthread_mutex_trylock(&lock)) {
    return 0;
  }
  pthread_mutex_unlock(&lock);
  pthread_t pth;
  pthread_create(&pth, NULL, &thread_read, this);
  return 1;
}

void CanInterface::write_can(msg_dvjet_sensor_e msgName, long long data) {
  // can_frame to_write;
  struct canfd_frame to_write;
  to_write.can_id = msg_array_dvjet_sensor[(int)msgName];
  to_write.len = 8;
  uint8_t signalArray[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  INT64_TO_ARRAY(data, signalArray); // Convert to array of bytes

  for (int i = 0; i < 8; i++) {
    to_write.data[i] = signalArray[i];
  }

  ssize_t bytes = write(sock, &to_write, sizeof(can_frame));

  if (bytes < 0)
    perror("CAN...'T WRITE (<0)");

  else if ((long unsigned int)bytes < sizeof(can_frame))
    perror("CAN...'T WRITE (<size)");
}

} // namespace jetson_interface
} // namespace utfr_dv
