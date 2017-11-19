#ifndef __IMU_TRANSFORMER_HPP__
#define __IMU_TRANSFORMER_HPP__

#include "../lcmtypes/imu_t.hpp"
#include "../lcmtypes/state_t.hpp"
#include <lcm/lcm-cpp.hpp>

class IMUTransformer 
{

public:
	IMUTransformer()
  : last_yaw(0) { }

	void handleIMU(const lcm::ReceiveBuffer * rbuf,
						 const std::string & chan,
						 const common::LCM::types::imu_t * imu_data);

	void handleSLAMState(const lcm::ReceiveBuffer * rbuf,
						 const std::string & chan,
						 const common::LCM::types::state_t * state_data);

private:
  double last_yaw; 
  common::LCM::types::imu_t publish_imu;
  lcm::LCM lcm;
};

#endif
