#include "FakeCompass.hpp" 
#include "Utilities.hpp"
#include <algorithm>
#include <cmath>

using namespace std;

// Returns the difference between the initial angle and the angle of the current position from
// the GPS' north axis. Used to initialize FOG's initial angle
double FakeCompass::getNorthLocation(double initial_theta)
{
  // End angle is calculated relative to GPS north
  double end_angle = atan2(xy_coords.back().second, xy_coords.back().first);
  // Returns an offset used to zero out the FOG at north
  return (initial_theta - end_angle);
}

void FakeCompass::addGPS(const gps_t & gps_data)
{
  if(!coord_transformer.isInitialized())
  {
    coord_transformer.initialize(gps_data.latitude, gps_data.longitude);
  }
  xy_coords.push_back(coord_transformer.transform(gps_data.latitude, gps_data.longitude));
}

double FakeCompass::getDistFromOrigin() const
{
  double x = xy_coords.back().first;
  double y = xy_coords.back().second;
  return sqrt(x*x + y*y);
}
