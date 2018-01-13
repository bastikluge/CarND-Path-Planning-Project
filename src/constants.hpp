#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

// The max s value before wrapping around the track back to 0
extern const double MAX_S;

// Number of lanes in the track
extern const unsigned NUMBER_OF_LANES;

// The width in meters of one lane of the track
extern const double LANE_WIDTH_M;

// The time increment in seconds within the car is controlled by one sent path
extern const double TIME_INCREMENT_S;

// Enumeration of sensor fusion indices
enum ESensorFusionIndices
{
  SFI_ID = 0,
  SFI_X,
  SFI_Y,
  SFI_VX,
  SFI_VY,
  SFI_S,
  SFI_D
};

// Goal speed at which the car desires to travel
extern double const GOAL_SPEED_MPS;

#endif