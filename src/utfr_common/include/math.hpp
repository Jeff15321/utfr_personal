/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: math.hpp
* auth: Kelvin Cui
* auth: Trevor Foote
* desc: math common header
*/
#pragma once

// System Requirements
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <list>
using Eigen::MatrixXd;
#include <tuple>
#include <string>

// Message Requirements
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <utfr_msgs/msg/cone_map.hpp>
#include <utfr_msgs/msg/ego_state.hpp>
#include <utfr_msgs/msg/trajectory_point.hpp>

namespace utfr_dv {
namespace util {

#define EPS 1e-8

/*! Calculate if two input doubles are equal to higher precisions.
 *  Compares different between both doubles to the defined EPS value.
 *  @param[in] a, b doubles to be compared.
 *  @returns bool true if doubles are within EPS range
 */
bool doubleEQ(const double a, const double b);

/*! Convert Degrees to Radians
 *
 *  @param[in] rad double of angle in radians
 *  @returns double of angle in degrees
 */
double degToRad(const double rad);

/*! Convert Radians to Degrees
 *
 *  @param[in] deg double of angle in degrees
 *  @returns double of angle in radians
 */
double radToDeg(const double deg);

/*! Calculate equivalent radian value within 0 to 2pi period
 *  @param[in] rad double of angle in radians
 *  @returns double of angle in radians within 0 to 2pi period
 */
double wrapRad(const double rad);

/*! Calculate equivalent degree value within 0 to 360 period
 *  @param[in] deg double of angle in degrees
 *  @returns double of angle in degree within 0 to 360 period
 */
double wrapDeg(const double deg);

/*! Caluclate Euclidian Distance in 2D of two points.
 *  Wrapper function for eucldianDistance for geometry_msgs::Points.
 *
 * @param[in] a geometry_msgs::Point 1 of XY coordinate
 * @param[in] b geometry_msgs::Point 2 of XY coordiante
 * @returns double of eucldiian distance between a and b.
 */
double euclidianDistance2D(const geometry_msgs::msg::Point &a,
                           const geometry_msgs::msg::Point &b);

/*! Caluclate Euclidian Distance in 2D of two points.
 *  Wrapper function for eucldianDistance for geometry_msgs::Vector3.
 *
 * @param[in] a geometry_msgs::Point 1 of XY coordinate
 * @param[in] b geometry_msgs::Point 2 of XY coordiante
 * @returns double of eucldiian distance between a and b.
 */
double euclidianDistance2D(const geometry_msgs::msg::Vector3 &a,
                           const geometry_msgs::msg::Vector3 &b);

/*! Calculate Euclidian Distance between two points
 *
 *  @param[in] x1 double of x coordinate of first point
 *  @param[in] x2 double of x coordinate of second point
 *  @param[in] y1 double of y coordinate of first point
 *  @param[in] y2 double of y coordinate of second point
 *  @returns double of Euclidian Distance
 */
double euclidianDistance2D(const double x1, const double x2, const double y1,
                           const double y2);

/*! Caluclate the magnitude of a vector.
 *
 * @param[in] a geometry_msgs::Point of XY coordinate
 * @returns double of magnitude of vector a.
 */
double vectorMagnitude(const geometry_msgs::msg::Vector3 &a);

/*! Calculate factornial of n
 *  @param[in] n unsigned integer
 *  @returns factorial of n
 */
int factorial(const unsigned int n);

/*! finds if there difference between two doubles are within bounds of another
 *
 *  @param[in] a double first double
 *  @param[in] b double second double
 *  @param[in] c double doube to compare with the difference between a and b
 */
bool withinBounds(double a, double b, double bound);

/*! Convert Lidar Scan to a vector of Cone objects
 *  @param[in] scan sensor_msgs::msg::LaserScan of lidar scan in radians
 *  @param[in] range_threshold double of range threshold to filter out points
 *  @returns std::vector<utfr_msgs::msg::Cone> of cone objects
 */
std::vector<utfr_msgs::msg::Cone>
convertLidarToCoordinates(const sensor_msgs::msg::LaserScan &scan,
                          const double range_threshold);

/*! Convert Latitude, Longitude, Altitude to North, East, Down co-ordinates,
 * based off a datum point
 *  @param[in] target geometry_msgs::msg::Vector3&, containing the target point.
 * Order of Lat. (Decimal Degrees), Long. (Decimal Degrees), Alt. (M)
 *  @param[in] world_datum geometry_msgs::msg::Vector3&, containing the datum
 * point. Order of Lat. (Decimal Degrees), Long. (Decimal Degrees), Alt. (M)
 *  @returns geometry_msgs::msg::Vector3, containing the distance in meters from
 * the datum point. Order of North (M), East (M), Down (M)
 */

geometry_msgs::msg::Vector3
convertLLAtoNED(const geometry_msgs::msg::Vector3 &target,
                const geometry_msgs::msg::Vector3 &world_datum);

/*! Convert North, East, Down co-ordinates to Latitude, Longitude, Altitude
 *  based off a world datum point and direction vector
 *  @param[in] direction geometry_msgs::msg::Vector3&, contains the
 *  direction vector. Order of North (M), East (M), Down (M)
 *  @param[in] world_datum geometry_msgs::msg::Vector3&, containing the datum
 *  point. Order of Lat. (Decimal Degrees), Long. (Decimal Degrees), Alt. (M)
 *  @returns geometry_msgs::msg::Vector3, containing the target datum point
 *  in LLA co-ordinates. Order of Lat. (Decimal Degrees),
 *  Long. (Decimal Degrees), Alt. (M)
 */
geometry_msgs::msg::Vector3
convertNEDtoLLA(const geometry_msgs::msg::Vector3 &direction,
                const geometry_msgs::msg::Vector3 &world_datum);

/*! Return crosstrack error bewteen 2 consecutive waypoints
 *
 * @param[in] point1 trajectory point 1
 * @param[in] point2 trajectory point 2
 * @param[in] ego current vehicle ego state
 * @returns double value of crosstrack error between path and ego vehicle
 *
 */
double getCrosstrackError(const utfr_msgs::msg::TrajectoryPoint &point1,
                          const utfr_msgs::msg::TrajectoryPoint &point2,
                          const utfr_msgs::msg::EgoState &ego);

/*! Return linear least squares fitting of list of cone positions
 *
 * @param[in] cones list of cones
 * @returns vector of doubles of line y=mx+b constants
 *
 */
std::tuple<double, double>
accelLLS(const std::vector<utfr_msgs::msg::Cone> &cones);

/*! Return linear least squares fitting of list of cone positions for occupancy
 *
 * @param[in] cones list of cones
 * @returns vector of doubles of line y=mx+b constants
 *
 */
std::tuple<double, double>
accelLLSOccupancy(const std::vector<utfr_msgs::msg::Cone> &cones);

/*! Return linear least suqres fitting of list of cone positions
 *
 * @param[in] cones list of cones
 * @param[in] radius to fit the circle to
 * @returns tuple of 3 double of cicle's (center x, center y, radius)
 *
 */
std::tuple<double, double, double, double>
ransacCircleLSF(const std::vector<utfr_msgs::msg::Cone> &cones, double radius);

/*! Yaw to Quaternion
 *
 * @param[in] yaw double of yaw in radians
 * @returns geometry_msgs::msg::Quaternion of yaw
 */
geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);

/*! Quaternion to Yaw
 *
 * @param[in] q geometry_msgs::msg::Quaternion of yaw
 * @returns double of yaw in radians
 */
double quaternionToYaw(const geometry_msgs::msg::Quaternion &q);
/*! Determine if a cone is a large orange cone or not
 *
 * @param[in] coneID ID of cone
 * @returns bool true if cone is large orange cone, false if not
 */

bool isLargeOrangeCone(const uint coneID);

/*! Helper function for easier acess of postion, velocity and steering angle
 *
 *@param[in] eg EgoState to be acessed
 *@param[in] infoWanted a char* that tells the function what to acess either "pos_x","pos_y","vel_x","vel_y" or "steering_angle"
 *@returns returns requested information or -10000000000000000 is input is invalid
 */
float egoHelper(utfr_msgs::msg::EgoState eg,const char* infoWanted);

} // namespace util
} // namespace utfr_dv
