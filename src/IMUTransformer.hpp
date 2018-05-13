#ifndef __IMU_TRANSFORMER_HPP__
#define __IMU_TRANSFORMER_HPP__

#include "../lcmtypes/imu_t.hpp"
#include "../lcmtypes/slam_state_t.hpp"
#include "../lcmtypes/simulator_pose_t.hpp"
#include <lcm/lcm-cpp.hpp>

class IMUTransformer 
{

public:
	IMUTransformer() { }

	void handleIMU(const lcm::ReceiveBuffer * rbuf,
						 const std::string & chan,
						 const imu_t * imu_data);

	void handleSimPose(const lcm::ReceiveBuffer * rbuf,
						 const std::string & chan,
						 const simulator_pose_t * simulator_pose_data);


private:
  lcm::LCM lcm;
  double sum_udot;
  double num_udot;
  double sum_vdot;
  double num_vdot;
  double sum_wdot;
  double num_wdot;
};

#endif
