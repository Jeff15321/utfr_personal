/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: canbus_util.hpp
* auth: Youssef Elhadad
* desc: CAN Interfacing Library for Jetson
*/

#include <rclcpp/rclcpp.hpp>

// CAN Libraries
#include <linux/can.h>
#include <linux/can/raw.h>
// Socket processing
#include <csignal>
#include <endian.h>
#include <net/if.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <bits/stdc++.h>
#include <unistd.h>

namespace utfr_dv {
namespace car_interface {

#ifndef _UTFR_CAN_JETSON_
#define _UTFR_CAN_JETSON_

enum dv_can_msg {
  // Sensors
  RBP = 0,
  FBP = 1,
  SPEEDFL = 2,
  SPEEDRL = 3,
  SPEEDFR = 21,
  SPEEDRR = 22,
  ImuY = 4,
  ImuX = 5,
  ImuZ = 6,
  ANGSENREC = 7,
  ANGSENTRA = 8,

  // Inverter
  MOTPOS = 9,
  APPS = 10,

  // FSG DV Log
  DVDrivingDynamics1 = 11,
  DVDrivingDynamics2 = 12,
  DVSystemStatus = 13,

  // Dv state sent from car
  DV_STATE = 14,

  // DV command + state sent from pc
  DV_COMMAND = 15,

  SetSTRMotorPos = 16,
  SetSTRMotorOrigin = 17,
  SetSTRMotorPosSpeedAcc = 18,
  StrMotorStatus = 19,

  // GPS Messages
  GPS_ERROR_CODE = 20, 
  GPS_SAMPLE_TIME = 21, 
  GPS_ORIENTATION = 22, // check if signals can be embedded 
  GPS_RATE_OF_TURN = 23, 
  GPS_ACCELERATION = 24, 
  GPS_LAT_LONG = 25, 
  GPS_ALT_ELLIP = 26, 
  GPS_VEL_XYZ = 27, 

  COMMANDED_TORQUE = 28, 
  ACTUAL_TORQUE = 29,

  COUNT = 30
};

typedef struct CAN_message_t {
  uint32_t id = 0;        // can identifier
  uint16_t timestamp = 0; // FlexCAN time when message arrived
  uint8_t idhit = 0;      // filter that id came from
  struct {
    bool extended = 0; // identifier is extended (29-bit)
    bool remote = 0;   // remote transmission request packet type
    bool overrun = 0;  // message overrun
    bool reserved = 0;
  } flags;
  uint8_t len = 8;      // length of data
  uint8_t buf[8] = {0}; // data
  int8_t mb = 0;        // used to identify mailbox reception
  uint8_t bus =
      0; // used to identify where the message came from when events() is used.
  bool seq = 0; // sequential frames
} CAN_message_t;

typedef struct CAN_signal_t {
  const uint8_t startBit = 0; 
  uint8_t len = 8; 
  const bool littleEndian = true; 
  const bool sign = false; 
  const float scale = 1.0; 
  const float offset = 0.0; 
  const float min = 0; 
  const float max = 10000; 
} CAN_signal_t; 

// From UTFR_CAN_TEENSY
// Little Endian
#define ARRAY_TO_INT64(array)                                                  \
  ((array[0]) | ((uint64_t)array[1] << 8) | ((uint64_t)array[2] << 16) |       \
   ((uint64_t)array[3] << 24) | ((uint64_t)array[4] << 32) |                   \
   ((uint64_t)array[5] << 40) | ((uint64_t)array[6] << 48) |                   \
   ((uint64_t)array[7] << 56))

// Big Endian
#define ARRAY_TO_INT64_BE(array)((array[7]) |\
                                ((uint64_t)array[6] << 8) |\
                                ((uint64_t)array[5] << 16) |\
                                ((uint64_t)array[4] << 24) |\
                                ((uint64_t)array[3] << 32) |\
                                ((uint64_t)array[2] << 40) |\
                                ((uint64_t)array[1] << 48) |\
                                ((uint64_t)array[0] << 56))

#define INT64_TO_ARRAY(num, array)                                             \
  do {                                                                         \
    array[0] = num & 0xFF;                                                     \
    array[1] = (num >> 8) & 0xFF;                                              \
    array[2] = (num >> 16) & 0xFF;                                             \
    array[3] = (num >> 24) & 0xFF;                                             \
    array[4] = (num >> 32) & 0xFF;                                             \
    array[5] = (num >> 40) & 0xFF;                                             \
    array[6] = (num >> 48) & 0xFF;                                             \
    array[7] = (num >> 56) & 0xFF;                                             \
  } while (0)

#define INT64_TO_ARRAY_REVERSE(num, array)                                     \
  do {                                                                         \
    array[7] = num & 0xFF;                                                     \
    array[6] = (num >> 8) & 0xFF;                                              \
    array[5] = (num >> 16) & 0xFF;                                             \
    array[4] = (num >> 24) & 0xFF;                                             \
    array[3] = (num >> 32) & 0xFF;                                             \
    array[2] = (num >> 40) & 0xFF;                                             \
    array[1] = (num >> 48) & 0xFF;                                             \
    array[0] = (num >> 56) & 0xFF;                                             \
  } while (0)

#endif

class CanInterface {
public:
  /*! Connect to CAN and enforce filter with CAN IDs,
   *
   *  @param[in] canline the can port to use
   *  @return true if works, false if connection failed
   */
  bool connect(const char *canline);

  /*! Get CAN Frame
   *
   *  @return the last read can data frame from can_id
   */
  int get_can(dv_can_msg msgName);

  /*! Read current CAN Frame
   *
   *  @return read for CAN (Blocking)
   */
  int read_can();

  /*! Write a CAN Frame to the CAN Bus
   *
   *  @param[in] to_write CAN Frame to push into CAN Bus
   */
  void write_can(dv_can_msg msgName, long long signalData);

  /*! Get CAN signals and messages with little endian.
   *
   *  @return data received by the DV computer.
   */
  float getSignal(dv_can_msg msgName, uint8_t startBit, uint8_t sigLength, bool sign, float scale);

  /*! Get CAN signals and messages with big endian.
   *
   *  @return data received by the DV computer.
   */
  float getSignalBE(dv_can_msg msgName, uint8_t startBit, uint8_t sigLength, bool sign, float scale);

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
  void setSignal(canfd_frame *to_send, dv_can_msg msgName, uint8_t startBit, uint8_t sigLength, float scale, double data);

  /*! Send CAN messages over the CAN bus.
   *
   *  @brief message is sent over CAN bus in little endian format. 
   */
  void sendSignal(canfd_frame to_write);

  // private:
  sockaddr_can addr;
  ifreq ifr;
  struct sigaction sa;

  int sock;
  int receive;
  int signal;
  pthread_mutex_t lock;
  pthread_mutex_t readlock;
  std::map<int, struct canfd_frame> messages;
};

using CanInterfaceUPtr = std::unique_ptr<CanInterface>;
using CanInterfaceSPtr = std::shared_ptr<CanInterface>;

} // namespace car_interface
} // namespace utfr_dv
