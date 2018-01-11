#ifndef MATH_HELPER_HPP
#define MATH_HELPER_HPP

#include <vector>
#include <math.h>

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

/**
 * @brief Calculates the euclidean distance from (x1, y1) to (x2, y2)
 */
double distance(double x1, double y1, double x2, double y2);

/**
 * @brief Returns the index of the closest waypoint within the map.
 *        Here, the waypoints describe the road to be followed.
 *        Note that the closest waypoint might be behind the vehicle.
 */
int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

/**
 * @brief Returns the index of the next waypoint (in vehicle direction) within the map.
 *        Here, the waypoints describe the road to be followed.
 */
int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

/**
 * @brief Transform from Cartesian x,y coordinates to Frenet s,d coordinates.
 *        The returned vector contains 2 entries {frenet_s,frenet_d}.
 */
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

/**
 * @brief Transform from Frenet s,d coordinates to Cartesian x,y
 *        The returned vector contains 2 entries {x,y}
 */
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

#endif // MATH_HELPER_HPP