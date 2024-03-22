/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: center_path_node.hpp
* auth: Justin Lim
* desc: center path node header
*/
#pragma once

// ROS2 Requirements
#include <rclcpp/rclcpp.hpp>

// System Requirements
#include <algorithm>
#include <chrono>
#include <deque>
#include <fstream>
#include <functional>
#include <queue>
#include <sstream>   // std::stringstream
#include <stdexcept> // std::runtime_error
#include <string>
#include <vector>

// Message Requirements
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/time.hpp>
#include <utfr_msgs/msg/cone_detections.hpp>
#include <utfr_msgs/msg/cone_map.hpp>
#include <utfr_msgs/msg/ego_state.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>
#include <utfr_msgs/msg/parametric_spline.hpp>
#include <utfr_msgs/msg/system_status.hpp>
#include <utfr_msgs/msg/target_state.hpp>
#include <utfr_msgs/msg/trajectory_point.hpp>

// UTFR Common Requirements
#include <utfr_common/frames.hpp>
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>

// Delaunay Requirements:
#ifdef EPS
#undef EPS
#endif
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
// #include <CGAL/draw_triangulation_2.h>

// Misc Requirements:
using std::placeholders::_1; // for std::bind

namespace utfr_dv {
namespace center_path {

class CenterPathNode : public rclcpp::Node {
public:
  /*! Constructor, calls loadParams, initPublishers and initTimers.
   */
  CenterPathNode();
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  // typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
  typedef K::Point_2 Point;

  typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, K> Vb;
  typedef CGAL::Triangulation_data_structure_2<Vb> Tds;
  typedef CGAL::Delaunay_triangulation_2<K, Tds> Delaunay;
  typedef Delaunay::Vertex_handle Vertex_handle;
  
  /*! This function tries to get global waypoints if possible and sets path_ */
  void GlobalWaypoints();
  
  /*! This function reads waypoints from a file
  * @param path the file path for the waypoints to read
  * @returns vector of the waypoints read
  */
  std::vector<std::pair<double,double>> getWaypoints(std::string path);
  
  /*! This function finds the next waypoint to follow and sends the current path
      to the controller
  */
  void nextWaypoint();
  
  /*! This function generates the transform by using the centres of the skipad 
      circle and the intersection of the circle
  */
  void createTransform();

  /*! This function transforms points from one frame of reference to cone coords
  * @param point the point to be transformed
  * @returns point transformed using the skidpad transform
  */
  std::pair<double,double> transformWaypoint(const std::pair<double,double> &point);
  
  /*! This function returns the left and right skidpad circle centres 
      respectively if they are valid. If only the right one is valid, the left 
      one is calculated. If none are valid it returns NAN.
  * @returns <left_x, left_y, right_x, right_y>
  */
  std::tuple<double,double,double,double> getCentres();
  
  /*! This function returns most feasible skidpad centres based on cone map
  * @returns <left_x, left_y, right_x, right_y>
  */
  std::tuple<double,double,double,double> skidpadCircleCentres();
  
  /*! This function calculates the circle of best fit with the radius for the 
      cones that satisfy the inlier count
  * @param cones the cones to find circle of best fit
  * @param radius the radius of the circle
  * @param inlier_count the number of cones the circle should contain at least
  * @returns <centre_x, centre_y>
  */
  std::pair<double,double> circleCentre(std::vector<utfr_msgs::msg::Cone> &cones, double radius, int inlier_count);

private:
  /*! Initialize and load params from config.yaml:
   */
  void initParams();

  /*! Initialize Subscribers:
   */
  void initSubscribers();

  /*! Initialize Publishers:
   */
  void initPublishers();

  void initEvent();

  void missionCB(const utfr_msgs::msg::SystemStatus &msg);

  /*! Initialize Timers:
   */
  void initTimers();

  /*! Initialize Sector:
   */
  void initSector();

  /*! Setup Heartbeat message with appropriate module name and update rate.
   */
  void initHeartbeat();

  /*! Send Heartbeat on every timer loop.
   *
   *  @param[in] status current module status, using Heartbeat status enum.
   */
  void publishHeartbeat(const int status);

  /*! EgoState Subscriber Callback:
   */
  void egoStateCB(const utfr_msgs::msg::EgoState &msg);

  /*! ConeMap Subscriber Callback:
   */
  void coneMapCB(const utfr_msgs::msg::ConeMap &msg);

  /*! ConeDetections Subscriber Callback:
   */
  void coneDetectionsCB(const utfr_msgs::msg::ConeDetections &msg);

  /*! Accel Timer Callback:
   */
  void timerCBAccel();

  /*! Skidpad Timer Callback:
   */
  void timerCBSkidpad();

  /*! Autocross Timer Callback:
   */
  void timerCBAutocross();

  /*! Trackdrive Timer Callback:
   */
  void timerCBTrackdrive();

  void timerCBEBS();

  void timerCBAS();

  /*! Function for sorting cones
   */
  bool coneDistComparitor(const utfr_msgs::msg::Cone &a,
                          const utfr_msgs::msg::Cone &b);

  /*! Accel line fitting
   * This function returns m and c for the line y=mx+c using cone detections
     completely colorblind
   */
  std::vector<double> getAccelPath();

  double midpointCostFunction(
      std::vector<int> nodes,
      const std::vector<CGAL::Point_2<CGAL::Epick>> &midpoints,
      std::vector<std::pair<CGAL::Point_2<CGAL::Epick>, unsigned int>>
          all_cones,
      std::vector<utfr_msgs::msg::Cone> yellow_cones,
      std::vector<utfr_msgs::msg::Cone> blue_cones,
      std::vector<std::pair<int, int>> midpoint_index_to_cone_indices);

  std::vector<CGAL::Point_2<CGAL::Epick>> getBestPath();

  /*! SkidPad Fit Function:
   */
  void skidPadFit(const utfr_msgs::msg::ConeDetections &cone_detections,
                  const utfr_msgs::msg::EgoState &msg);

  /*! Publish Fitted Skidpad Path Lines:
   */
  void publishLine(double m_left, double m_right, double c_left, double c_right,
                   double x_min, double x_max, double thickness);

  std::tuple<std::vector<CGAL::Point_2<CGAL::Epick>>, std::vector<double>,
             std::vector<double>>
  BezierPoints(std::vector<CGAL::Point_2<CGAL::Epick>> midpoints);

  /*! Skidpad Lap Counter
   */
  void skidpadLapCounter();

  /*! Autox/Trackdrive Lap Counter
   */
  void trackdriveLapCounter();

  std::tuple<double, double, double, double, double, double> skidpadMain();
  std::tuple<double, double, double, double, double, double> skidpadRight();
  std::tuple<double, double, double, double, double, double> skidpadLeft();

  /*! Initialize global variables:
   */
  double update_rate_;
  std::string event_;

  const double centre_distance_ = 9.125; // skidpad centres to track centre dist
  const int small_circle_cones_ = 16; // number of cones in small circle
  const int big_circle_cones_ = 13; // number of cones in large circle
  std::unique_ptr<MatrixXd> skidpadTransform_{nullptr};
  std::vector<std::pair<double,double>> waypoints;
  std::vector<std::tuple<double,double,double>> visited;
  bool global_path_;

  double small_radius_;
  double big_radius_;
  double threshold_radius_;
  int threshold_cones_;
  int curr_sector_ = 0;
  bool lock_sector_;
  bool cones_detected_ = false;
  bool found_4_large_orange;
  rclcpp::Time last_time;
  bool accel_sector_increase;
  int detections_in_row_ = 0;

  utfr_msgs::msg::EgoState::SharedPtr ego_state_{nullptr};
  utfr_msgs::msg::ConeMap::SharedPtr cone_map_{nullptr};
  utfr_msgs::msg::ConeDetections::SharedPtr cone_detections_{nullptr};
  geometry_msgs::msg::Point reference_point_;

  rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr
      ego_state_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::ConeMap>::SharedPtr cone_map_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::ConeDetections>::SharedPtr
      cone_detection_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::SystemStatus>::SharedPtr
      mission_subscriber_;

  rclcpp::Publisher<utfr_msgs::msg::ParametricSpline>::SharedPtr
      center_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      accel_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      skidpad_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      skidpad_path_publisher_2_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      skidpad_path_publisher_avg_;
  rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      delaunay_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      first_midpoint_path_publisher_;
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::Time ros_time_;

  utfr_msgs::msg::TargetState target_;
  utfr_msgs::msg::SystemStatus::SharedPtr status_{nullptr};
  utfr_msgs::msg::Heartbeat heartbeat_;
};
} // namespace center_path
} // namespace utfr_dv
