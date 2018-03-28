#ifndef __GPS_TRANSFORMER_HPP__
#define __GPS_TRANSFORMER_HPP__

#include "../lcmtypes/gps_t.hpp"
#include "../lcmtypes/state_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include "CoordTransformer.hpp"


class GPSTransformer 
{

public:
	GPSTransformer() 
	: last_coord_perfect_gps{0, 0}, last_coord_gps{0, 0} { }

	void handleGPS(const lcm::ReceiveBuffer * rbuf,
						 const std::string & chan,
						 const common::LCM::types::gps_t * gps_data);

private:
  std::pair<double, double> last_coord_perfect_gps;
  std::pair<double, double> last_coord_gps;
  common::LCM::types::gps_t perfect_gps;
  common::LCM::types::gps_t gps;
  CoordTransformer perfect_gps_transformer;
  CoordTransformer gps_transformer;
  lcm::LCM lcm;
};

#endif
