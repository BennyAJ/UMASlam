#ifndef __IMU_TRANSFORMER_HPP__
#define __IMU_TRANSFORMER_HPP__

#include "../lcmtypes/imu_t.hpp"
#include "../lcmtypes/global_imu_t.hpp"
#include "../lcmtypes/slam_state_t.hpp"
#include <lcm/lcm-cpp.hpp>

class IMUTransformer 
{

public:
	IMUTransformer() { }

	void handleIMU(const lcm::ReceiveBuffer * rbuf,
						 const std::string & chan,
						 const imu_t * imu_data);


private:
  lcm::LCM lcm;
};

#endif
