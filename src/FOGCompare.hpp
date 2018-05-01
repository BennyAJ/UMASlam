#ifndef __FOG_COMPARE_HPP__
#define __FOG_COMPARE_HPP__

#include "../lcmtypes/fog_t.hpp"
#include <lcm/lcm-cpp.hpp>


class FOGCompare
{

public:
	FOGCompare() 
	: last_fog_data(0) { }

	void handleFOG(const lcm::ReceiveBuffer * rbuf,
						 const std::string & chan,
						 const fog_t * fog_data);

private:
  double last_fog_data;
  fog_t perfect_fog;
  fog_t fog;
  lcm::LCM lcm;
};

#endif
