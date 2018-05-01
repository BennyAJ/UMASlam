#ifndef __GPS_COMPARE_HPP__
#define __GPS_COMPARE_HPP__

#include "../lcmtypes/gps_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include "CoordTransformer.hpp"


class GPSCompare
{

public:
	GPSCompare() 
	: last_coord_perfect_gps{0, 0}, last_coord_gps{0, 0} { }

	void handleGPS(const lcm::ReceiveBuffer * rbuf,
						 const std::string & chan,
						 const gps_t * gps_data);

private:
  std::pair<double, double> last_coord_perfect_gps;
  std::pair<double, double> last_coord_gps;
  gps_t perfect_gps;
  gps_t gps;
  CoordTransformer perfect_gps_transformer;
  CoordTransformer gps_transformer;
  lcm::LCM lcm;
};

#endif
