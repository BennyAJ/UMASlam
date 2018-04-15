#ifndef __IMU_COMPARE_HPP__
#define __IMU_COMPARE_HPP__

#include "../lcmtypes/imu_t.hpp"
#include "../lcmtypes/state_t.hpp"
#include <lcm/lcm-cpp.hpp>


class IMUCompare
{

public:
	IMUCompare() 
	: last_perfect_imu_info{0, 0}, last_imu_info{0, 0} { }

	void handleIMU(const lcm::ReceiveBuffer * rbuf,
						 const std::string & chan,
						 const common::LCM::types::imu_t * imu_data);

private:
  // first double is udot and second is vdot
  std::pair<double, double> last_perfect_imu_info;
  std::pair<double, double> last_imu_info;
  common::LCM::types::imu_t perfect_imu;
  common::LCM::types::imu_t imu;
  lcm::LCM lcm;
};

#endif
