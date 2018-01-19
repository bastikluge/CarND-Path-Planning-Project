#include "trajectory_helper.hpp"
#include <math.h>
#include <iostream>

using namespace std;

bool isSafeLane(double check_dt, double check_s, double safe_s, bool in_driving_dir, unsigned lane_idx, const SensorFusionCollection &sensor_fusion)
{
  for ( unsigned idxCar=0; idxCar<sensor_fusion.size(); idxCar++ )
  {
    // check if car is in my lane
    double other_d(sensor_fusion[idxCar][SFI_D]);
    if ( (lane_idx * LANE_WIDTH_M < other_d) && (other_d <= (lane_idx+1) * LANE_WIDTH_M) )
    {
      double other_vx(sensor_fusion[idxCar][SFI_VX]);
      double other_vy(sensor_fusion[idxCar][SFI_VY]);
      double other_s (sensor_fusion[idxCar][SFI_S]);
      double other_v(sqrt(other_vx*other_vx + other_vy*other_vy));

      // check if car is in front and in safe distance
      double other_check_s = other_s + check_dt*other_v;
      if ( in_driving_dir )
      {
        if ( (other_check_s >= check_s) && (other_check_s <= check_s + safe_s) )
        {
          std::cout << "           Car " << sensor_fusion[idxCar][SFI_ID] << " ahead on lane " << lane_idx << " is too close!\n";
          return false;
        }
      }
      else
      {
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