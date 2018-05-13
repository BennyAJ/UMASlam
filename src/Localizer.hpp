#ifndef __SLAM_LOCALIZER_HPP__
#define __SLAM_LOCALIZER_HPP__

#include "../lcmtypes/slam_state_t.hpp"
#include "../lcmtypes/gps_t.hpp"
#include "../lcmtypes/imu_t.hpp"
#include "../lcmtypes/global_imu_t.hpp"
#include "CoordTransformer.hpp"
#include "Utilities.hpp"
#include <lcm/lcm-cpp.hpp>
#include <string>
#include <vector>
#include <random>
#include <utility>

struct Vec2 {
  double north;
  double east;
};

class Localizer
{
public:
  Localizer()
    : imu_state_initialized(false),
    last_accel({0, 0}), 
    last_vel({0, 0}), 
    current_vel({0, 0}) { }

  void handleGPSData(const lcm::ReceiveBuffer * rbuf,
             const std::string & chan,
             const gps_t * gps_data);

  void handleIMUData(const lcm::ReceiveBuffer * rbuf,
             const std::string & chan,
             const global_imu_t * imu_data);

  void publishGPSState(const gps_t * gps_data);

private:
  lcm::LCM l;

  slam_state_t imu_state;
  bool imu_state_initialized;
  int64_t last_imu_utime;
  Vec2 last_accel;
  Vec2 last_vel;
  Vec2 current_vel;

  double last_imu_angle;

  CoordTransformer coord_transformer;
};

#endif
