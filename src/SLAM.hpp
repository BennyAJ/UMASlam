#ifndef __UMA_SLAM_HPP__
#define __UMA_SLAM_HPP__

#include "Localizer.hpp"
#include "FakeCompass.hpp"
#include "../lcmtypes/fog_t.hpp"
#include "../lcmtypes/gps_t.hpp"
#include "../lcmtypes/compass_t.hpp"
#include "../lcmtypes/slam_state_t.hpp"

#include <string>
#include <mutex>
#include <iostream>
#include <lcm/lcm-cpp.hpp>

class Slam
{
  Localizer localizer;
  lcm::LCM llcm;

  size_t num_mapped_scans;

public:
  Slam();

  void handleState(const lcm::ReceiveBuffer * rbuf,
           const std::string & chan,
           const slam_state_t * state);

  void handleGPSData(const lcm::ReceiveBuffer * rbuf,
             const std::string & chan,
             const gps_t * gps_data);

  void handleFOGData(const lcm::ReceiveBuffer * rbuf,
             const std::string & chan,
             const fog_t * fog_data);

  void handleCompassData(const lcm::ReceiveBuffer * rbuf,
             const std::string & chan,
             const compass_t * compass_data);

  void handleIMUData(const lcm::ReceiveBuffer * rbuf,
             const std::string & chan,
             const imu_t * imu_data);

  void run();


private:
  FakeCompass fake_compass;

  bool fog_initialized;

  // Angle of north relative to current position in radians
  bool compass_initialized;
  double compass_north;

  // same as above, but for imu
  bool imu_initialized;
  double imu_north;

};
#endif
