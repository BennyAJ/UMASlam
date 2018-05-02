#ifndef __PERFECT_STATE_GENERATOR_HPP__
#define __PERFECT_STATE_GENERATOR_HPP__

#include "CoordTransformer.hpp"
#include "../lcmtypes/gps_t.hpp"
#include "../lcmtypes/fog_t.hpp"
#include "../lcmtypes/slam_state_t.hpp"
#include <lcm/lcm-cpp.hpp>

class PerfectStateGenerator 
{

public:
	PerfectStateGenerator() 
    : coord_transformer(), 
      slam_state_set(false),
      fog_set(false),
      gps_set(false)
  { 
  }

	void handleFOG(const lcm::ReceiveBuffer * rbuf,
						 const std::string & chan,
						 const fog_t * fog_data);

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
  bool fog_set;
  bool gps_set;
};

#endif
