#ifndef TRAJECTORY_HELPER_HPP
#define TRAJECTORY_HELPER_HPP

#include "constants.hpp"

/**
 * Calculates a safe distance in meter from a speed in meter per second
 */
double calcSafeDistance(double v);

/**
 * Helper method to check whether a lane is safe at a given location for the input sensor fusion data
 * @param[in] check_dt        time increment to the future in seconds, at which the lane safety shall be checked
 * @param[in] check_s         location of the planning vehicle (Frenet s) at time = now + check_dt
 * @param[in] check_v         speed of the planning vehicle in meter per second at time = now + check_dt
 * @param[in] safety_factor   factor, which will be used to determine a safe distance from vehicle speed
 * @param[in] in_driving_dir  flag, which indicates whether vehicles in front shall be checked (true) or vehicles behind shall be checked (false)
 * @param[in] lane_idx        index of lane, for which safety shall be checked
 * @param[in] sensor_fusion   sensor fusion data, which contains the information of all vehicles' states close to the planning vehicle
 * @return true if and only driving on the lane is safe
 */
bool isSafeLane(double check_dt, double check_s, double check_v, double safety_factor, bool in_driving_dir, unsigned lane_idx, const SensorFusionCollection &sensor_fusion);

/**
 * Helper method to check whether a lane is safe (and for which distance) at a given location for the input sensor fusion data
 * @param[in] check_dt        time increment to the future in seconds, at which the lane safety shall be checked
 * @param[in] check_s         location of the planning vehicle (Frenet s) at time = now + check_dt
 * @param[in] in_driving_dir  flag, which indicates whether vehicles in front shall be checked (true) or vehicles behind shall be checked (false)
 * @param[in] lane_idx        index of lane, for which safety shall be checked
 * @param[in] sensor_fusion   sensor fusion data, which contains the information of all vehicles' states close to the planning vehicle
 * @return safe distance
 */
double getMinDistance(double check_dt, double check_s, bool in_driving_dir, unsigned lane_idx, const SensorFusionCollection &sensor_fusion);

#endif // TRAJECTORY_HELPER_HPP