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

  COUNT = 20
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

// From UTFR_CAN_TEENSY
#define ARRAY_TO_INT64(array)                                                  \
  ((array[0]) | ((uint64_t)array[1] << 8) | ((uint64_t)array[2] << 16) |       \
   ((uint64_t)array[3] << 24) | ((uint64_t)array[4] << 32) |                   \
   ((uint64_t)array[5] << 40) | ((uint64_t)array[6] << 48) |                   \
   ((uint64_t)array[7] << 56))

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
