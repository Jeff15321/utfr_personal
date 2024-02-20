/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: sensor_util.hpp
* auth: Daniel Asadi
* desc: Sensor data parsing over canbus header
*/

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

namespace utfr_dv {
namespace car_interface {

/**
 * @brief Retrieves the steering angle sensor data.
 */
void CarInterface::getSteeringAngleSensorData();

/**
 * @brief Retrieves the motor speed data.
 */
void CarInterface::getMotorSpeedData();

/**
 * @brief Retrieves the motor torque data.
 */
void CarInterface::getMotorTorqueData();

/**
 * @brief Retrieves the service brake data.
 */
void CarInterface::getServiceBrakeData();

/**
 * @brief Retrieves the wheelspeed sensor data.
 */
void CarInterface::getWheelspeedSensorData();

/**
 * @brief Retrieves the IMU data.
 */
void CarInterface::getIMUData();

/**
 * @brief Retrieves the sensor CAN data.
 */
void CarInterface::getSensorCan();

/**
 * @brief Retrieves the DV state.
 */
void CarInterface::getDVState();

} // namespace car_interface
} // namespace utfr_dv
