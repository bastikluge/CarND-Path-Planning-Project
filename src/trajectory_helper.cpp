#include "trajectory_helper.hpp"
#include <math.h>
#include <iostream>

using namespace std;

double calcSafeDistance(double v)
{
  return 0.5*v*3.6; // German rule "halber Tacho": dist_in_m = 0.5 * speed_in_kmh
}

bool isSafeLane(double check_dt, double check_s, double check_v, double safety_factor, bool in_driving_dir, unsigned lane_idx, const SensorFusionCollection &sensor_fusion)
{
  for ( unsigned idxCar=0; idxCar<sensor_fusion.size(); idxCar++ )
  {
    // check if car is in considered lane
    double other_d(sensor_fusion[idxCar][SFI_D]);
    if ( (lane_idx * LANE_WIDTH_M < other_d) && (other_d <= (lane_idx+1) * LANE_WIDTH_M) )
    {
      double other_vx(sensor_fusion[idxCar][SFI_VX]);
      double other_vy(sensor_fusion[idxCar][SFI_VY]);
      double other_s (sensor_fusion[idxCar][SFI_S]);
      double other_v(sqrt(other_vx*other_vx + other_vy*other_vy));

      // check if car is in safe distance
      double other_check_s = other_s + check_dt*other_v;
      if ( in_driving_dir )
      {
        double safe_s = safety_factor * calcSafeDistance(check_v);
        if ( (other_check_s >= check_s) && (other_check_s <= check_s + safe_s) )
        {
          std::cout << "           Car " << sensor_fusion[idxCar][SFI_ID] << " ahead on lane " << lane_idx << " is too close!\n";
          return false;
        }
      }
      else
      {
        double safe_s = safety_factor * calcSafeDistance(other_v);
        if ( (other_check_s <= check_s) && (other_check_s >= check_s - safe_s) )
        {
          std::cout << "           Car " << sensor_fusion[idxCar][SFI_ID] << " behind on lane " << lane_idx << " is too close!\n";
          return false;
        }
      }
    }
  }
  return true;
}


double getMinDistance(double check_dt, double check_s, bool in_driving_dir, unsigned lane_idx, const SensorFusionCollection &sensor_fusion)
{
  double min_distance(1000.0);
  for ( unsigned idxCar=0; idxCar<sensor_fusion.size(); idxCar++ )
  {
    // check if car is in considered lane
    double other_d(sensor_fusion[idxCar][SFI_D]);
    if ( (lane_idx * LANE_WIDTH_M < other_d) && (other_d <= (lane_idx+1) * LANE_WIDTH_M) )
    {
      double other_vx(sensor_fusion[idxCar][SFI_VX]);
      double other_vy(sensor_fusion[idxCar][SFI_VY]);
      double other_s (sensor_fusion[idxCar][SFI_S]);
      double other_v(sqrt(other_vx*other_vx + other_vy*other_vy));

      // calculate minimum distance
      double other_check_s = other_s + check_dt*other_v;
      if ( in_driving_dir )
      {
        if ( other_check_s >= check_s )
        {
          if ( other_check_s - check_s < min_distance )
          {
            min_distance = other_check_s - check_s;
          }
        }
      }
      else
      {
        if ( other_check_s <= check_s )
        {
          if ( check_s - other_check_s < min_distance )
          {
            min_distance = other_check_s - check_s;
          }
        }
      }
    }
  }
  return min_distance;
}