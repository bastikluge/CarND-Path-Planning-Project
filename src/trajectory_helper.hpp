#ifndef TRAJECTORY_HELPER_HPP
#define TRAJECTORY_HELPER_HPP

#include "constants.hpp"

bool isSafeLane(double check_dt, double check_s, double safe_s, bool in_driving_dir, unsigned lane_idx, const SensorFusionCollection &sensor_fusion);

#endif // TRAJECTORY_HELPER_HPP