#ifndef __PERFECT_STATE_GENERATOR_HPP__
#define __PERFECT_STATE_GENERATOR_HPP__

#include "CoordTransformer.hpp"
#include "../lcmtypes/gps_t.hpp"
#include "../lcmtypes/imu_t.hpp"
#include "../lcmtypes/slam_state_t.hpp"
#include <lcm/lcm-cpp.hpp>

class PerfectStateGenerator 
{

public:
	PerfectStateGenerator() 
    : coord_transformer(), 
      slam_state_set(false),
      imu_set(false),
      gps_set(false)
  { 
  }

	void handleIMU(const lcm::ReceiveBuffer * rbuf,
						 const std::string & chan,
						 const imu_t * imu_data);

	void handleGPS(const lcm::ReceiveBuffer * rbuf,
						 const std::string & chan,
						 const gps_t * gps_data);

	void handleSlamState(const lcm::ReceiveBuffer * rbuf,
						 const std::string & chan,
						 const slam_state_t * slam_state_data);

  bool readyToPublish();

  void publishPerfectState();

private:
  lcm::LCM lcm;
  slam_state_t perfect_state;
  CoordTransformer coord_transformer;

  bool slam_state_set;
  bool imu_set;
  bool gps_set;
};

#endif
