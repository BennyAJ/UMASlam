#ifndef __SLAM_LOCALIZER_HPP__
#define __SLAM_LOCALIZER_HPP__

#include "../lcmtypes/gps_t.hpp"
#include "CoordTransformer.hpp"
#include "Utilities.hpp"
#include <lcm/lcm-cpp.hpp>
#include <string>
#include <vector>
#include <random>

class Localizer
{
public:
  void handleGPSData(const lcm::ReceiveBuffer * rbuf,
             const std::string & chan,
             const gps_t * gps_data);

private:
  void publishPose() const;
  std::pair<double, double> last_coord;
  double last_yaw;
  int64_t last_utime;

  CoordTransformer coord_transformer;
};

#endif
