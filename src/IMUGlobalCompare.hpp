#ifndef __IMU_COMPARE_HPP__
#define __IMU_COMPARE_HPP__

#include "../lcmtypes/imu_t.hpp"
#include "../lcmtypes/state_t.hpp"
#include <lcm/lcm-cpp.hpp>


class IMUGlobalCompare
{

public:
	IMUGlobalCompare() 
	: last_sim_imu_info{0, 0} { }

	void handleIMU(const lcm::ReceiveBuffer * rbuf,
						 const std::string & chan,
						 const common::LCM::types::imu_t * imu_data);

private:
  std::pair<double, double> last_sim_imu_info;
  lcm::LCM lcm;
};

#endif
