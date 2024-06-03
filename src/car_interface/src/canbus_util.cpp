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

    {(uint8_t)dv_can_msg::SPEEDFL, 0x005}, // Wheel speed
    {(uint8_t)dv_can_msg::SPEEDRL, 0x006}, // Wheel speed
    {(uint8_t)dv_can_msg::SPEEDFR, 0x007}, // Wheel speed
    {(uint8_t)dv_can_msg::SPEEDRR, 0x008}, // Wheel speed

    {(uint8_t)dv_can_msg::ImuY, 0x174}, // IMU data
    {(uint8_t)dv_can_msg::ImuX, 0x178}, // IMU data
    {(uint8_t)dv_can_msg::ImuZ, 0x17C}, // IMU data

    {(uint8_t)dv_can_msg::ANGSENREC, 0x2B0}, // Steering angle sensor value
    {(uint8_t)dv_can_msg::ANGSENTRA, 0x7C0}, // Setup steering angle sensor

    // Motor/inverter
    {(uint8_t)dv_can_msg::MOTPOS, 0x0A5},           // Motor speed
    {(uint8_t)dv_can_msg::COMMANDED_TORQUE, 0x0C0}, // torque command
    {(uint8_t)dv_can_msg::ACTUAL_TORQUE, 0x0B0},    // torque feedback

    {(uint8_t)dv_can_msg::DVDrivingDynamics1, 0x500}, // FSG DV logging
    {(uint8_t)dv_can_msg::DVDrivingDynamics2, 0x501}, // FSG DV logging
    {(uint8_t)dv_can_msg::DVSystemStatus, 0x502},     // FSG DV logging

    {(uint8_t)dv_can_msg::FULL_AS_STATE, 0x504}, // DV state from car
    {(uint8_t)dv_can_msg::DV_COMP_STATE, 0x506}, // DV PC state + control cmd
    {(uint8_t)dv_can_msg::MISSION, 0x508},       // DV Mission select from dial

    {(uint8_t)dv_can_msg::SetSTRMotorPos,
     0x0000040F}, // Set Pos on Steering motor
    {(uint8_t)dv_can_msg::SetSTRMotorOrigin,
     0x0000050F}, // Set Origin on Steering motor
    {(uint8_t)dv_can_msg::SetSTRMotorPosSpeedAcc,
     0x0000060F}, // Set Pos, speed, and accel on Steering motor
    {(uint8_t)dv_can_msg::StrMotorInfo, 0x0000290F}, // Status of Steering motor
    // GPS CAN
    {(uint8_t)dv_can_msg::GPS_ERROR_CODE, 0x001},
    {(uint8_t)dv_can_msg::GPS_SAMPLE_TIME,
     0x010}, // sample time vs UTC time? check ros driver
    {(uint8_t)dv_can_msg::GPS_RPY, 0x019},
    {(uint8_t)dv_can_msg::GPS_ORIENTATION, 0x023},
    {(uint8_t)dv_can_msg::GPS_LAT_LONG, 0x071},
    {(uint8_t)dv_can_msg::GPS_ALT_ELLIP, 0x072},
    {(uint8_t)dv_can_msg::GPS_VEL_XYZ, 0x076},
    {(uint8_t)dv_can_msg::GPS_ACCELERATION, 0x034},
    {(uint8_t)dv_can_msg::GPS_RTK_STATUS, 0x009}, 
    {(uint8_t)dv_can_msg::STR_MOTOR_CMD, 0x0000040F}, 
    {(uint8_t)dv_can_msg::DV_COMMANDED, 0x0D0}
    };

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

uint64_t CanInterface::get_can(dv_can_msg msgName) { // TODO: remove
  // while(pthread_mutex_trylock(&readlock)){;}
  uint64_t result;
  if (msgName == dv_can_msg::ANGSENREC) {
    result =
        (int)((messages[dv_can_msg_map[(uint8_t)msgName]].data[0]) |
              (((messages[dv_can_msg_map[(uint8_t)msgName]].data[1]) << 8)));
  } else if (msgName == dv_can_msg::MOTPOS) {
    result =
        (int)((messages[dv_can_msg_map[(uint8_t)msgName]].data[2]) |
              (((messages[dv_can_msg_map[(uint8_t)msgName]].data[3]) << 8)));
  } /*else if (msgName == dv_can_msg::APPS) {
    result =
        (int)((messages[dv_can_msg_map[(uint8_t)msgName]].data[0]) |
              (((messages[dv_can_msg_map[(uint8_t)msgName]].data[1]) << 8)));
  }*/
  else if (msgName == dv_can_msg::StrMotorInfo) {
    result =
        (int)((messages[dv_can_msg_map[(uint8_t)msgName]].data[1]) |
              (((messages[dv_can_msg_map[(uint8_t)msgName]].data[0]) << 8)));
  } else if (msgName == dv_can_msg::MISSION) {
    // SOMETHING IS STILL WRONG HERE EVEN THOUGH WE ARE RECIEVING PROPERLY FROM
    // BELOW PRINT STATEMENT
    result = (int)(messages[dv_can_msg_map[(int)msgName]].data[0]);
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
    if (recieved.can_id & CAN_EFF_FLAG) {
      canNode->messages[recieved.can_id & CAN_EFF_MASK] = recieved;
    } else {
      canNode->messages[recieved.can_id] = recieved;
    }
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

void CanInterface::write_can(dv_can_msg msgName, long long data, bool byteWise) {
  // can_frame to_write;
  struct canfd_frame to_write;
  to_write.can_id = dv_can_msg_map[(int)msgName];
  to_write.len = 8;
  uint8_t signalArray[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  if (byteWise) {
    // Little Endian (initially)
    INT64_TO_ARRAY_REVERSE(data, signalArray);
  } else {
    // Little Endian: reverse data bits.
    // Since many signals are not in multiple of 8s, cannot simply move everything in bytes.
    unsigned long long littleEndianData = 0; 

    // Bit Reversal
    for (int i = 0; i < 64; i++) {
      littleEndianData = littleEndianData | ((data & 1) << (64 - i - 1)); 
      data = data >> 1;
    }

    INT64_TO_ARRAY(littleEndianData, signalArray); // Convert to array of bytes
  }
  
  // The steering motor uses EXTENDED CAN, and sends in Little Endian.
  if (msgName == dv_can_msg::SetSTRMotorPos ||
      msgName == dv_can_msg::SetSTRMotorOrigin ||
      msgName == dv_can_msg::SetSTRMotorPosSpeedAcc || 
      msgName == dv_can_msg::STR_MOTOR_CMD) {
    to_write.can_id = dv_can_msg_map[(int)msgName] | (CAN_EFF_FLAG);
    INT64_TO_ARRAY_REVERSE(data, signalArray); // Little Endian (bytes reversed)
  }

  for (uint8_t i = 0; i < 8; i++) {
    to_write.data[i] = signalArray[i];
  }

  ssize_t bytes = write(sock, &to_write, sizeof(can_frame));

  if (bytes < 0)
    perror("CAN...'T WRITE (<0) HERE");

  else if ((long unsigned int)bytes < sizeof(can_frame))
    perror("CAN...'T WRITE (<size)");
}

/*! Get CAN signals and messages with little endian.
 *
 * @return data received by the DV computer.
 */
float CanInterface::getSignal(dv_can_msg msgName, uint8_t startBit,
                              uint8_t sigLength, bool sign, float scale) {
  uint64_t signalData =
      ARRAY_TO_INT64(messages[dv_can_msg_map[(int)msgName]].data);

  // Bit mask to filter out desired bits.
  // Bitwise & will be used (the bitmask will be a consecutive series of 1s and
  // will be 0s elsewhere).
  uint64_t mask = 0;
  for (uint8_t i = 0; i < sigLength; i++) {
    mask = (mask << 1) + 1;
  }

  signalData = signalData >> startBit;
  signalData = signalData & mask;

  // Signed representation.
  int64_t signSignalData;
  if (sign) {
    uint64_t maxValue = pow(2, sigLength - 1) - 1;

    if (signalData > maxValue) {
      signSignalData = (~signalData + 1) & mask;

      return -signSignalData * scale;
    }
  }

  return signalData * scale;
}

/*! Get CAN signals and messages with big endian.
 *
 * Note that CAN messages are at most 8 bytes long (16 nibbles).
 * This function can only be used for 11-bit standard CAN protocols.
 *
 * @param msgName CAN message identifier.
 * @param startBit start bit of desired signal.
 * @param sigLength length of signal in bits.
 * @param sign true if signed, false if unsigned.
 * @param scale amount of scaling applied to the CAN signal.
 *
 * @return data received by the DV computer.
 */
float CanInterface::getSignalBE(dv_can_msg msgName, uint8_t startBit,
                                uint8_t sigLength, bool sign, float scale) {
  // Convert 8 bytes into a 64 bit integer in big endian format.
  uint64_t signalData =
      ARRAY_TO_INT64_BE(messages[dv_can_msg_map[(int)msgName]].data);

  // Bit mask to filter out desired bits.
  // Bitwise & will be used (the bitmask will be a consecutive series of 1s and
  // will be 0s elsewhere).
  uint64_t mask = 0;
  for (uint8_t i = 0; i < sigLength; i++) {
    mask = (mask << 1) + 1;
  }

  signalData = signalData >> (64 - startBit - sigLength);
  signalData = signalData & mask;

  // Signed numbers should have 2's complement applied to it, only if it is
  // above the maximum value able to be represented by a number of binary
  // digits. Negative Representation = Bitwise NOT + 1 (2's Complement)
  int64_t signSignalData;
  if (sign) {
    uint64_t maxValue = pow(2, sigLength - 1) - 1;

    if (signalData > maxValue) {
      // Mask the result to the signal length.
      signSignalData = (~signalData + 1) & mask;

      return -signSignalData * scale;
    }
  }

  return signalData * scale;
}

/*
canfd_frame CanInterface::setSignal(canfd_frame to_send, dv_can_msg msgName,
                             uint8_t startBit, uint8_t sigLength, float scale,
                             double data) {
  to_send.can_id = dv_can_msg_map[(int)msgName];
  to_send.len = 8;
  uint8_t signalArray[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  // Take raw data and scale it if necessary.
  uint64_t can_data = (uint64_t)(data / scale);

  // Convert to byte format.
  uint64_t mask = 0xFF; // 8 Bit Mask

  for (int i = startBit / 8 + sigLength / 8 - 1; i >= startBit / 8; i--) {
    uint8_t byte = can_data & mask;
    signalArray[i] = byte;

    can_data = can_data >> 8;
  }

  // Set data within data frame.
  for (uint8_t i = startBit / 8; i < startBit / 8 + sigLength / 8; i++) {
    (to_send.data)[i] = signalArray[i];
  }

  // Works: problem with sendSignal
  return to_send;
}
*/

/*! Send CAN messages over the CAN bus.
 *
 *  @brief message is sent over CAN bus in little endian format.
 */
void CanInterface::sendSignal(canfd_frame to_write) {
  // Check if a double pointer for to_write is needed. 
  ssize_t bytes = write(sock, &to_write, sizeof(canfd_frame));

  const char * error_msg = std::to_string(to_write.can_id).c_str();

  perror("CAN ID: ");
  perror(error_msg);

  if (bytes < 0)
    perror("CAN...'T WR ITE SEND SIGNAL (<0)");

  else if ((long unsigned int)bytes < sizeof(can_frame))
    perror("CAN...'T WRITE SEND SIGNAL (<size)");
}

/*! Set CAN signals and messages to BUS using little endian.
 *
 * @param canfd_frame address of frame used to set data and CAN ID.
 * @param msgName DV CAN message name.
 * @param startBit little Endian format.
 * @param sigLength number of bits.
 * @param scale amount the data has been scaled by.
 * @param data data (e.g. an angle) that has undergone scaling.
 * @return data received by the DV computer.
 */
uint64_t CanInterface::setSignal(uint64_t to_send, uint8_t startBit, uint8_t sigLength, double scale, double data) {
  uint64_t can_data = (uint64_t) (data / scale); 

  uint64_t mask = pow(2, sigLength) - 1; 

  to_send = to_send | ((can_data & mask) << (64 - startBit - sigLength));

  return to_send;
}

} // namespace car_interface
} // namespace utfr_dv
