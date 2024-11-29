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
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
#include <utfr_msgs/msg/lap_time.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

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
  * @returns <centre_x, centre_y, radius, threshold>
  */
  std::tuple<double,double,double,double> circleCentre(std::vector<utfr_msgs::msg::Cone> &cones, double radius, int inlier_count);

  std::tuple<double,double,double,double> getSkidpadCircleCentresColourblind();

  std::tuple<double, double, double, double> getCentresColourblind();

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

  /*! Initialize Transforms:
   */
  void initTransforms();

  /*! Initialize event subscriber to read from system status: 
   */
  void initEvent();
  
  /*! Set event based on system status: 
   *  @param[in] msg system status message
   */
  void missionCB(const utfr_msgs::msg::SystemStatus &msg);

  /*! Initialize TimerCB functions depending on event:
   */
  void initTimers();

  /*! Initialize Sector count based on event:
   */
  void initSector();

  /*! Setup Heartbeat message with appropriate module name and update rate.
   */
  void initHeartbeat();

  /*! Send Heartbeat on every timer loop.
   *  @param[in] status int, current module status, using Heartbeat status enum.
   */
  void publishHeartbeat(const int status);

  /*! EgoState Subscriber Callback:
   * @param[in] msg utfr_msgs::msg::EgoState incoming ego state msg
   */
  void egoStateCB(const utfr_msgs::msg::EgoState &msg);

  /*! ConeMap Subscriber Callback:
   * @param[in] msg utfr_msgs::msg::ConeMap incoming cone map msg
   */
  void coneMapCB(const utfr_msgs::msg::ConeMap &msg);

  /*! ConeDetections Subscriber Callback:
    * @param[in] msg utfr_msgs::msg::ConeDetections incoming cone detections msg
   */
  void coneDetectionsCB(const utfr_msgs::msg::ConeDetections &msg);

  /**
   * ConeMap Closure Subscriber Callback:
   * @param[in] msg utfr_msgs::msg::ConeDetections incoming cone detections msg
   */
  void coneMapClosureCB(const std_msgs::msg::Bool &msg);

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

  /*! EBS Test Timer Callback:
   */
  void timerCBEBS();

  /*! Autonomous System Test Timer Callback
   */
  void timerCBAS();

  /*! Function for sorting cones
   * @param[in] a utfr_msgs::msg::Cone, cone a
   * @param[in] b utfr_msgs::msg::Cone, cone b
   * @param[out] bool, true if cone a is closer than cone b
   */
  bool coneDistComparitor(const utfr_msgs::msg::Cone &a,
                          const utfr_msgs::msg::Cone &b);

  /*! Accel line fitting:
   * This function returns m and c for the line y=mx+c using cone detections
     completely colorblind
   * @param[out] std::pair<double, double>, m and c for the line y=mx+c
   */
  utfr_msgs::msg::ParametricSpline getAccelPath();

  /*! Cost function to sort midpoint paths:
   * @param[in] nodes std::vector<int>, list of integers representing the midpoints indexes
   * @param[in] midpoints std::vector<CGAL::Point_2<CGAL::Epick>>, list of midpoints
   * @param[in] all_cones std::vector<std::pair<CGAL::Point_2<CGAL::Epick>, unsigned int>>, list of all cones
   * @param[in] yellow_cones std::vector<utfr_msgs::msg::Cone>, list of yellow cones
   * @param[in] blue_cones std::vector<utfr_msgs::msg::Cone>, list of blue cones
   * @param[in] midpoint_index_to_cone_indices std::vector<std::pair<int, int>>, list of pairs of midpoints and cone indices
   * @param[out] double, cost of the path
   */
  double midpointCostFunction(
      const std::vector<int> &nodes,
      const std::vector<CGAL::Point_2<CGAL::Epick>> &midpoints,
      const std::vector<std::pair<CGAL::Point_2<CGAL::Epick>, unsigned int>>
          &all_cones,
      const std::vector<utfr_msgs::msg::Cone> &yellow_cones,
      const std::vector<utfr_msgs::msg::Cone> &blue_cones,
      const std::vector<std::pair<int, int>> &midpoint_index_to_cone_indices);
  
  /*! Get the best path from the midpoints:
   * @param[out] std::vector<CGAL::Point_2<CGAL::Epick>>, list of midpoints
   */
  utfr_msgs::msg::ParametricSpline getBestPath();

  /*! SkidPad Fit Function. Interntally calculates path based on sector count:
   */
  void skidPadFit();

  /*! Publish Fitted Skidpad Path Lines:
   * @param[in] m_left double, left line slope
   * @param[in] m_right double, right line slope
   * @param[in] c_left double, left line y-intercept
   * @param[in] c_right double, right line y-intercept
   * @param[in] x_min double, minimum x value
   * @param[in] x_max double, maximum x value
   */
  void publishLine(double m_left, double m_right, double c_left, double c_right,
                   double x_min, double x_max);
  
  /*! Given a bunch of midpoints, fit a bezier curve:
   * @param[in] midpoints std::vector<CGAL::Point_2<CGAL::Epick>>, list of midpoints
   * @param[out] std::tuple<std::vector<CGAL::Point_2<CGAL::Epick>>, std::vector<double>, std::vector<double>>, parametric x[t] and y[t]
   */
  std::tuple<std::vector<CGAL::Point_2<CGAL::Epick>>, std::vector<double>,
             std::vector<double>>
  BezierPoints(std::vector<CGAL::Point_2<CGAL::Epick>> midpoints);

  /*! Skidpad Lap Counter:
   */
  void skidpadLapCounter();

  void skidpadLapCounterColourblind();

  bool checkPassedDatum(const utfr_msgs::msg::EgoState reference,
                        const utfr_msgs::msg::EgoState &current);

  utfr_msgs::msg::EgoState getSkidpadDatum(const utfr_msgs::msg::ConeMap &cone_map);

  /*! Autox/Trackdrive Lap Counter
   */
  void trackdriveLapCounter();


  /**
   * Trackdrive get datum finds the point where the lap starts given a conemap. used in global lapcounter. 
   * 
   * @param cone_map 
   * @return utfr_msgs::msg::EgoState 
   */
  utfr_msgs::msg::EgoState getTrackDriveDatum(const utfr_msgs::msg::ConeMap &cone_map);

  
  /*! Skidpad path finder when there are enough blue and yellow cones to fit a line
   * @param[out] std::tuple<double, double, double, double, double, double>, x center left, y center left, radius left, x center right, y center right, radius right
   */
  std::tuple<double, double, double, double, double, double> skidpadMain();

  /*! Skidpad path finder using only right (yellow) cones for when turning right
   * @param[out] std::tuple<double, double, double, double, double, double>, x center left, y center left, radius left, x center right, y center right, radius right
   */
  std::tuple<double, double, double, double, double, double> skidpadRight();

  /*! Skidpad path finder using only left (blue) cones for when turning left
   * @param[out] std::tuple<double, double, double, double, double, double>, x center left, y center left, radius left, x center right, y center right, radius right
   */
  std::tuple<double, double, double, double, double, double> skidpadLeft();

  /*! Function to calculate and publish lap times
   */
  void publishLapTime();

  /*! Converts something from global to local coordinates
   * @param[in] x, y, yaw in global coordinates
   * @param[out] x, y in local coodrinates
   */
  std::vector<double> globalToLocal(double x, double y); 

  /*! Checks if a point is within the hemisphere of the car
   * @param[in] x, y, point of the cone
   * @param[in] r, radius of the hempishere
   * @param[out] true if within the hemisphere, false if not
   */
  bool hempishere(double x, double y, double r);

  /*! Takes a list of cones and given the current position of the car, returns which cones are in the hemisphere
   * @param[in] list of cones
   * @param[in] radius we want
   * @param[out] list of cones in himesphere
   */
  std::vector<utfr_msgs::msg::Cone> getConesInHemisphere(std::vector<utfr_msgs::msg::Cone> cones, double r);

  /*! Initialize global variables:
   */
  enum EventState{
    ERROR = 0,
    // Acceleration FSM
    ACCEL_STRAIGHT = 1,
    ACCEL_FINISHED = 2,
    // Skidpad FSM
    SMALL_ORANGE_STRAIGHT = 10,
    LARGE_ORANGE_STRAIGHT = 11,
    RIGHT_CIRCLE_FIRST = 12,
    RIGHT_CIRCLE_SECOND = 13,
    LEFT_CIRCLE_FIRST = 14,
    LEFT_CIRCLE_SECOND = 15,
    END_SMALL_ORANGE = 16,
    SKIDPAD_FINISHED = 17,
    // Autocross FSM
    AUTOCROSS_LAP_1 = 20,
    AUTCROSS_FINISHED = 21,
    // Trackdrive FSM
    TRACKDRIVE_LAP_1 = 30,
    TRACKDRIVE_LAP_2 = 31,
    TRACKDRIVE_LAP_3 = 32,
    TRACKDRIVE_LAP_4 = 33,
    TRACKDRIVE_LAP_5 = 34,
    TRACKDRIVE_LAP_6 = 35,
    TRACKDRIVE_LAP_7 = 36,
    TRACKDRIVE_LAP_8 = 37,
    TRACKDRIVE_LAP_9 = 38,
    TRACKDRIVE_LAP_10 = 39,
    TRACKDRIVE_FINISHED = 40,
  };
  
  void velocityCB(const geometry_msgs::msg::Vector3Stamped &msg);
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr vel_subscriber_;

  double update_rate_;
  std::string event_;

  const double centre_distance_ = 9.125; // skidpad centres to track centre dist
  const int small_circle_cones_ = 16; // number of cones in small circle
  const int big_circle_cones_ = 13; // number of cones in large circle
  std::unique_ptr<MatrixXd> skidpadTransform_{nullptr};
  std::vector<std::pair<double,double>> waypoints;
  std::vector<std::tuple<double,double,double>> visited;
  bool global_path_;
  double max_velocity_;

  bool event_set_ = false;
  double small_radius_;
  double big_radius_;
  double threshold_radius_;
  int threshold_cones_;
  int curr_sector_ = 0;
  bool lock_sector_;
  bool cones_detected_ = false;
  bool found_4_large_orange;
  rclcpp::Time last_time;
  rclcpp::Time last_switch_time;
  int last_sector = 0;
  float best_lap_time = 0.0;
  float last_lap_time = 0.0;
  int detections_in_row_ = 0;
  bool use_mapping_ = false;
  bool use_autocross_for_accel_ = false;
  double base_lookahead_distance_;
  double lookahead_scaling_factor_;
  int as_state = 0;

  bool loop_closed_ = false;

  double average_distance_to_cones_ = 10.0;

  double datum_last_local_x_ = 0;

  double total_distance_traveled_ = 0.0;

  double switch_distance = 0.0;

  bool colourblind_;

  utfr_msgs::msg::EgoState::SharedPtr ego_state_{nullptr};
  utfr_msgs::msg::ConeMap::SharedPtr cone_map_local_{nullptr};
  utfr_msgs::msg::ConeMap::SharedPtr cone_map_global_{nullptr};
  utfr_msgs::msg::ConeDetections::SharedPtr cone_detections_{nullptr};
  geometry_msgs::msg::Point reference_point_;

  rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr
      ego_state_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::ConeMap>::SharedPtr cone_map_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      cone_map_closure_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::ConeDetections>::SharedPtr
      cone_detection_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::SystemStatus>::SharedPtr
      mission_subscriber_;

  rclcpp::Publisher<utfr_msgs::msg::ParametricSpline>::SharedPtr
      center_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr
      center_point_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      accel_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      skidpad_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      skidpad_path_publisher_2_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      skidpad_path_publisher_avg_;
  rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr delauny_midpoint_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      delaunay_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      first_midpoint_path_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::LapTime>::SharedPtr
      lap_time_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      lap_datum_publisher_;
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::Time ros_time_;
  
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  utfr_msgs::msg::TargetState target_;
  utfr_msgs::msg::SystemStatus::SharedPtr status_{nullptr};
  utfr_msgs::msg::Heartbeat heartbeat_;
};
} // namespace center_path
} // namespace utfr_dv
