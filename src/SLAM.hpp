#ifndef __UMA_SLAM_HPP__
#define __UMA_SLAM_HPP__

#include "Localizer.hpp"
#include "../lcmtypes/gps_t.hpp"

#include <string>
#include <iostream>
#include <lcm/lcm-cpp.hpp>

class Slam
{

  size_t num_mapped_scans;

public:
  Slam();

  void handleGPSData(const lcm::ReceiveBuffer * rbuf,
             const std::string & chan,
             const gps_t * gps_data);

  void handleIMUData(const lcm::ReceiveBuffer * rbuf,
             const std::string & chan,
             const global_imu_t * imu_data);

  void run();

private:
  Localizer localizer;
  lcm::LCM llcm;
};
#endif
