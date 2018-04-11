#ifndef __IMU_TRANSFORMER_HPP__
#define __IMU_TRANSFORMER_HPP__

#include "../lcmtypes/imu_t.hpp"
#include "../lcmtypes/slam_state_t.hpp"
#include <lcm/lcm-cpp.hpp>

class IMUTransformer 
{

public:
	IMUTransformer()
  : last_yaw(0) { }

	void handleIMU(const lcm::ReceiveBuffer * rbuf,
						 const std::string & chan,
						 const imu_t * imu_data);

	void handleSLAMState(const lcm::ReceiveBuffer * rbuf,
						 const std::string & chan,
						 const slam_state_t * slam_state);

private:
  double last_yaw; 
  imu_t publish_imu;
  lcm::LCM lcm;
};

#endif
