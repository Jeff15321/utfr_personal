/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: canbus_util.cpp
* auth: Youssef Elhadad
* desc: CAN Interfacing Library for Jetson
*/

#include <canbus_util.hpp>

namespace utfr_dv {
namespace car_interface {

std::map<uint8_t, canid_t> dv_can_msg_map{
    {(uint8_t)dv_can_msg::RBP, 0x008}, // Rear brake pressure
    {(uint8_t)dv_can_msg::FBP, 0x010}, // Front brake pressure
    {(uint8_t)dv_can_msg::SPEEDFL, 0x012},
    {(uint8_t)dv_can_msg::SPEEDRL, 0x013}, // Wheel speed
    {(uint8_t)dv_can_msg::ImuY, 0x174},
    {(uint8_t)dv_can_msg::ImuX, 0x178},
    {(uint8_t)dv_can_msg::ImuZ, 0x17C},
    {(uint8_t)dv_can_msg::ANGSENREC, 0x2B0}, // Steering angle sensor value
    {(uint8_t)dv_can_msg::ANGSENTRA, 0x7C0}, // Setup steering angle sensor
    {(uint8_t)dv_can_msg::MOTPOS, 0x0A5},    // Motor speed
    {(uint8_t)dv_can_msg::APPS, 0x004},      // Motor torque

    {(uint8_t)dv_can_msg::DVDrivingDynamics1, 0x500}, // FSG DV logging
    {(uint8_t)dv_can_msg::DVDrivingDynamics2, 0x501}, // FSG DV logging
    {(uint8_t)dv_can_msg::DVSystemStatus, 0x502},     // FSG DV logging

    {(uint8_t)dv_can_msg::DV_STATE, 0x504}, // DV state from car

    {(uint8_t)dv_can_msg::DV_COMMAND, 0x506}, // DV PC state + control cmd

    {(uint8_t)dv_can_msg::SetMotorPos, 0x0000040F}, // Set Pos on Steering motor
    {(uint8_t)dv_can_msg::SetMotorOrigin,
     0x0000050F}, // Set Origin on Steering motor
    {(uint8_t)dv_can_msg::SetMotorPosSpeedAcc,
     0x0000060F}, // Set Pos, speed, and accel on Steering motor
    {(uint8_t)dv_can_msg::MotorStatus, 0x0000290F}}; // Set Status of motor

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

int CanInterface::get_can(dv_can_msg msgName) {
  // while(pthread_mutex_trylock(&readlock)){;}
  int result;
  if (msgName == dv_can_msg::ANGSENREC) {
    result = (int)((messages[dv_can_msg_map[(int)msgName]].data[0]) |
                   (((messages[dv_can_msg_map[(int)msgName]].data[1]) << 8)));
  } else if (msgName == dv_can_msg::MOTPOS) {
    result = (int)((messages[dv_can_msg_map[(int)msgName]].data[2]) |
                   (((messages[dv_can_msg_map[(int)msgName]].data[3]) << 8)));
  } else if (msgName == dv_can_msg::APPS) {
    result = (int)((messages[dv_can_msg_map[(int)msgName]].data[0]) |
                   (((messages[dv_can_msg_map[(int)msgName]].data[1]) << 8)));
  } else {
    result = ARRAY_TO_INT64(
        messages[dv_can_msg_map[(int)msgName]]
            .data); // messages[dv_can_msg_map[(int)msgName]].data;
  }

  // pthread_mutex_unlock(&readlock);
  return result;
}

static void *thread_read(void *node) {
  utfr_dv::car_interface::CanInterface *canNode =
      (utfr_dv::car_interface::CanInterface *)node;
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

void CanInterface::write_can(dv_can_msg msgName, long long data) {
  // can_frame to_write;
  struct canfd_frame to_write;
  to_write.can_id = dv_can_msg_map[(int)msgName];
  if (msgName == dv_can_msg::SetMotorPos ||
      msgName == dv_can_msg::SetMotorOrigin ||
      msgName == dv_can_msg::SetMotorPosSpeedAcc)
    to_write.can_id =
        dv_can_msg_map[(int)msgName] & (CAN_EFF_MASK | CAN_EFF_FLAG);
  to_write.len = 8;
  uint8_t signalArray[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  INT64_TO_ARRAY(data, signalArray); // Convert to array of bytes

  for (uint8_t i = 0; i < 8; i++) {
    to_write.data[i] = signalArray[i];
  }

  ssize_t bytes = write(sock, &to_write, sizeof(can_frame));

  if (bytes < 0)
    perror("CAN...'T WRITE (<0)");

  else if ((long unsigned int)bytes < sizeof(can_frame))
    perror("CAN...'T WRITE (<size)");
}

} // namespace car_interface
} // namespace utfr_dv
