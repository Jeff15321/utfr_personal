/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: math.cpp
* auth: Kelvin Cui
* auth: Trevor Foote
* desc: math common functions
*/

#include <iostream>
#include <math.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <utfr_msgs/msg/cone_map.hpp>

namespace utfr_dv {
namespace util {

bool doubleEQ(const double a, const double b) {
  double abs_diff = fabs(a - b);
  return (abs_diff < EPS);
}

double radToDeg(const double rad) { return rad * 180 / M_PI; }

double degToRad(const double deg) { return deg * M_PI / 180; }

double wrapRad(const double rad) {
  if (rad >= 0 && rad < 2 * M_PI) {
    return rad;
  } else if (rad < 0) {
    return wrapRad(rad + 2 * M_PI);
  } else {
    return wrapRad(rad - 2 * M_PI);
  }
}

double wrapDeg(const double deg) {
  if (deg >= 0 && deg < 360) {
    return deg;
  } else if (deg < 0) {
    return wrapDeg(deg + 360);
  } else {
    return wrapDeg(deg - 360);
  }
}

double euclidianDistance2D(const geometry_msgs::msg::Point &a,
                           const geometry_msgs::msg::Point &b) {
  return euclidianDistance2D(a.x, b.x, a.y, b.y);
}

double euclidianDistance2D(const geometry_msgs::msg::Vector3 &a,
                           const geometry_msgs::msg::Vector3 &b) {
  return euclidianDistance2D(a.x, b.x, a.y, b.y);
}

double euclidianDistance2D(const double x1, const double x2, const double y1,
                           const double y2) {
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

int factorial(const unsigned int n) {
  if (n <= 1) {
    return 1;
  } else {
    return factorial(n - 1) * n;
  }
}

double vectorMagnitude(const geometry_msgs::msg::Vector3 &a) {
  return sqrt(pow(a.x, 2) + pow(a.y, 2));
}

std::vector<utfr_msgs::msg::Cone>
convertLidarToCoordinates(const sensor_msgs::msg::LaserScan &scan,
                          const double range_threshold) {
  // Initialize an empty vector to hold the cones
  std::vector<utfr_msgs::msg::Cone> cones;

  // Loop through all the ranges in the scan
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    // If the range is within a certain threshold, consider it a cone
    if (scan.ranges[i] > 0 && scan.ranges[i] < range_threshold) {
      // Convert the range and angle to x and y coordinates
      double x =
          scan.ranges[i] * std::cos(scan.angle_min + scan.angle_increment * i);
      double y =
          scan.ranges[i] * std::sin(scan.angle_min + scan.angle_increment * i);

      // Create a Cone object with the coordinates and an unknown type
      utfr_msgs::msg::Cone cone;
      cone.pos.x = x;
      cone.pos.y = y;
      cone.type = utfr_msgs::msg::Cone::UNKNOWN;

      // Add the cone to the vector
      cones.push_back(cone);
    }
  }

  return cones;
}

geometry_msgs::msg::Vector3
convertLLAtoNED(const geometry_msgs::msg::Vector3 &target,
                const geometry_msgs::msg::Vector3 &world_datum) {
  // Constants we need
  double semimajor_axis = 6378137.0;
  double semiminor_axis = 6356752.314245;

  double target_lat = degToRad(target.x);
  double target_lon = degToRad(target.y);
  double world_datum_lat = degToRad(world_datum.x);
  double world_datum_lon = degToRad(world_datum.y);

  // LLA -> NED conversion for both the target and the world datum
  double N = pow(semimajor_axis, 2) /
             sqrt((pow(semimajor_axis, 2) * pow(std::cos(target_lat), 2)) +
                  (pow(semiminor_axis, 2) * pow(std::sin(target_lat), 2)));
  double x_lla = (N + target.z) * std::cos(target_lat) * std::cos(target_lon);
  double y_lla = (N + target.z) * std::cos(target_lat) * std::sin(target_lon);
  double z_lla = (N * pow((semiminor_axis / semimajor_axis), 2) + target.z) *
                 std::sin(target_lat);

  N = pow(semimajor_axis, 2) /
      sqrt(pow(semimajor_axis, 2) * pow(std::cos(world_datum_lat), 2) +
           pow(semiminor_axis, 2) * pow(std::sin(world_datum_lat), 2));
  double x_lla0 = (N + world_datum.z) * std::cos(world_datum_lat) *
                  std::cos(world_datum_lon);
  double y_lla0 = (N + world_datum.z) * std::cos(world_datum_lat) *
                  std::sin(world_datum_lon);
  double z_lla0 =
      (N * pow((semiminor_axis / semimajor_axis), 2) + world_datum.z) *
      std::sin(world_datum_lat);

  // Find the distances between the two points
  double u = x_lla - x_lla0;
  double v = y_lla - y_lla0;
  double w = z_lla - z_lla0;

  double t = std::cos(world_datum_lon) * u + std::sin(world_datum_lon) * v;

  double east =
      -std::sin(world_datum_lon) * u + std::cos((world_datum_lon)) * v;
  double up = std::cos(world_datum_lat) * t + std::sin(world_datum_lat) * w;
  double north = -std::sin(world_datum_lat) * t + std::cos(world_datum_lat) * w;

  // Declare a Vector3 message to hold the NED coordinates
  geometry_msgs::msg::Vector3 ned;
  ned.x = north;
  ned.y = east;
  ned.z = -up;

  return ned;
}

double getCrosstrackError(const utfr_msgs::msg::TrajectoryPoint &point1,
                          const utfr_msgs::msg::TrajectoryPoint &point2,
                          const utfr_msgs::msg::EgoState &ego) {

  // get slope between trajectories
  double deltaX = point2.pos.x - point1.pos.x;
  // double deltaY = point2.pos.y - point1.pos.y, trajectorySlope;
  if (deltaX == 0)
    return std::abs(ego.pose.pose.position.x - point1.pos.x);

  double trajectorySlope =
      (point1.pos.y - point2.pos.y) / (point1.pos.x - point2.pos.x);

  if (trajectorySlope == 0)
    return std::abs(ego.pose.pose.position.y - point1.pos.y);

  double invertedSlope = -1 / trajectorySlope;

  // find constants of every line
  double bTrajectory = point1.pos.y - (trajectorySlope * point1.pos.x),
         bInverted = ego.pose.pose.position.y -
                     (invertedSlope * ego.pose.pose.position.x);
  double xIntersection =
      (bTrajectory - bInverted) / (invertedSlope - trajectorySlope);
  double yIntersection = (xIntersection * trajectorySlope) + bTrajectory;

  // Return Euclidean distance between intersection and car (lateral distance)
  return euclidianDistance2D(ego.pose.pose.position.x, xIntersection,
                             ego.pose.pose.position.y, yIntersection);
}

bool withinBounds(double a, double b, double bound) {
  if (sqrt(pow(a, 2) + pow(b, 2)) <= bound)
    return true;
  return false;
}

std::tuple<double, double>
accelLLS(const std::vector<utfr_msgs::msg::Cone> &cones) {
  int n = cones.size();
  MatrixXd A(n, 2);
  MatrixXd b(n, 1);
  for (int i = 0; i < n; i++) {
    A(i, 0) = cones[i].pos.y;
    A(i, 1) = 1;
    b(i, 0) = cones[i].pos.x;
  }
  MatrixXd At = A.transpose();
  MatrixXd res = (At * A).inverse() * (At * b);
  double m = res(0);
  double c = res(1);
  std::tuple<double, double> lineFit;
  lineFit = std::make_tuple(m, c);
  return lineFit;
}

std::tuple<double, double>
accelLLSOccupancy(const std::vector<utfr_msgs::msg::Cone> &cones) {
  int n = cones.size();
  MatrixXd A(n, 2);
  MatrixXd b(n, 1);
  for (int i = 0; i < n; i++) {
    A(i, 0) = cones[i].pos.x;
    A(i, 1) = 1;
    b(i, 0) = cones[i].pos.y;
  }
  MatrixXd At = A.transpose();
  MatrixXd res = (At * A).inverse() * (At * b);
  double m = res(0);
  double c = res(1);
  std::tuple<double, double> lineFit;
  lineFit = std::make_tuple(m, c);
  return lineFit;
}

geometry_msgs::msg::Vector3
convertNEDtoLLA(const geometry_msgs::msg::Vector3 &direction,
                const geometry_msgs::msg::Vector3 &world_datum) {
  // Constants we need
  double semimajor_axis = 6378137.0;
  double semiminor_axis = 6356752.31424518;

  // geodetic2ecef
  // Convert from degrees to radians
  double world_datum_lat = degToRad(world_datum.x);
  double world_datum_lon = degToRad(world_datum.y);

  double N = pow(semimajor_axis, 2) /
             sqrt(pow(semimajor_axis, 2) * pow(std::cos(world_datum_lat), 2) +
                  pow(semiminor_axis, 2) * pow(std::sin(world_datum_lat), 2));
  double x0_ecef = (N + world_datum.z) * std::cos(world_datum_lat) *
                   std::cos(world_datum_lon);
  double y0_ecef = (N + world_datum.z) * std::cos(world_datum_lat) *
                   std::sin(world_datum_lon);
  double z0_ecef =
      (N * pow(semiminor_axis / semimajor_axis, 2) + world_datum.z) *
      std::sin(world_datum_lat);

  // enu2uvw
  double t = std::cos(world_datum_lat) * (-direction.z) -
             std::sin(world_datum_lat) * direction.x;
  double dz_w = std::sin(world_datum_lat) * (-direction.z) +
                std::cos(world_datum_lat) * direction.x;

  double dx_u =
      std::cos(world_datum_lon) * t - std::sin(world_datum_lon) * direction.y;
  double dy_v =
      std::sin(world_datum_lon) * t + std::cos(world_datum_lon) * direction.y;

  // enu2ecf
  double x = x0_ecef + dx_u;
  double y = y0_ecef + dy_v;
  double z = z0_ecef + dz_w;

  // ecef2geodetic
  double r = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
  double E = sqrt(pow(semimajor_axis, 2) - pow(semiminor_axis, 2));

  double u = sqrt(0.5 * (pow(r, 2) - pow(E, 2)) +
                  0.5 * (std::hypot(pow(r, 2) - pow(E, 2), 2 * E * z)));

  double Q = std::hypot(x, y);
  double huE = std::hypot(u, E);

  double Beta;

  try {
    Beta = std::atan(huE / u * z / std::hypot(x, y));
  } catch (std::overflow_error &overflow_error) {
    if (abs(z) <= 1e-9) {
      Beta = 0;
    } else if (z > 0) {
      Beta = M_PI / 2;
    } else {
      Beta = -M_PI / 2;
    }
  }

  double dBeta =
      ((semiminor_axis * u - semimajor_axis * huE + pow(E, 2)) * sin(Beta)) /
      (semimajor_axis * huE * (1 / std::cos(Beta)) -
       pow(E, 2) * std::cos(Beta));

  Beta += dBeta;

  double target_lat =
      std::atan(semimajor_axis / semiminor_axis * std::tan(Beta));

  double lim_pi2;

  double eps = 2.22044604925e-16;
  try {
    lim_pi2 = M_PI / 2 - eps;
    target_lat = Beta >= lim_pi2 ? M_PI / 2 : target_lat;
    target_lat = Beta <= -lim_pi2 ? -M_PI / 2 : target_lat;
  } catch (std::exception &error) {
    // Pass
  }

  double target_lon = std::atan2(y, x);

  double cosBeta = std::cos(Beta);

  try {
    cosBeta = Beta >= lim_pi2 ? 0 : cosBeta;
    cosBeta = Beta <= -lim_pi2 ? 0 : cosBeta;
  } catch (std::exception &error) {
    // Pass
  }

  double target_alt =
      std::hypot(z - semiminor_axis * sin(Beta), Q - semimajor_axis * cosBeta);

  bool inside =
      (pow(x, 2) / pow(semimajor_axis, 2) + pow(y, 2) / pow(semimajor_axis, 2) +
           pow(z, 2) / pow(semiminor_axis, 2) <
       1);

  if (inside) {
    target_alt = -target_alt;
  }

  // Convert from radians to degrees
  target_lat = radToDeg(target_lat);
  target_lon = radToDeg(target_lon);

  // Declare a Vector3 message to hold the LLA coordinates
  geometry_msgs::msg::Vector3 lla;
  lla.x = target_lat;
  lla.y = target_lon;
  lla.z = target_alt;

  return lla;
}

std::tuple<double, double, double, double>
ransacCircleLSF(const std::vector<utfr_msgs::msg::Cone> &cones, double radius) {
  int n = cones.size();
  MatrixXd A(n, 3);
  // MatrixXd A(n,2);
  MatrixXd b(n, 1);

  for (int i = 0; i < n; i++) {
    A(i, 0) = 2.0 * cones[i].pos.y;
    A(i, 1) = 2.0 * cones[i].pos.x;
    A(i, 2) = -1.0;
    // b(i, 0) = pow((cones[i].pos.x),2)+pow((cones[i].pos.y),2);
    b(i, 0) =
        pow((cones[i].pos.x), 2) + pow((cones[i].pos.y), 2) - pow(radius, 2);
  }

  MatrixXd At = A.transpose();
  MatrixXd res = (At * A).inverse() * (At * b);
  double xc = res(0);
  double yc = res(1);

  double radiustot;
  for (int i = 0; i < n; i++) {
    radiustot +=
        sqrt(pow((xc - cones[i].pos.x), 2) + pow((yc - cones[i].pos.y), 2));
  }

  double radiusf = radiustot / n;

  std::tuple<double, double, double, double> circle;
  circle = std::make_tuple(xc, yc, radius, radiusf);
  return circle;
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.w = cos(yaw * 0.5);
  q.x = 0;
  q.y = 0;
  q.z = sin(yaw * 0.5);
  return q;
}

double quaternionToYaw(const geometry_msgs::msg::Quaternion &q) {
  double yaw;
  yaw =
      atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return yaw;
}

bool isLargeOrangeCone(uint8 coneID) {
    if (coneID == 4) {
        return true;
    }
    return false;
}

} // namespace util
} // namespace utfr_dv
