#include "IMUTransformer.hpp"
#include "Constants.hpp"
#include "Utilities.hpp"
#include <cmath>

using namespace std;

void IMUTransformer::handleIMU(const lcm::ReceiveBuffer * rbuf,
                    const string & chan,
                    const imu_t * imu_data)
{
  publish_imu.utime = imu_data->utime;
  // new_x = cos(theta) * old_x - sin(theta) * old_y
  publish_imu.vdot = cos(last_yaw) * imu_data->vdot - sin(last_yaw) * imu_data->udot;
  // new_y = sin(theta) * old_x + cos(theta) * old_y
  publish_imu.udot = sin(last_yaw) * imu_data->vdot + cos(last_yaw) * imu_data->udot;

  lcm.publish(TRANSFORMED_IMU_CHANNEL, &publish_imu);
}


void IMUTransformer::handleSLAMState(const lcm::ReceiveBuffer * rbuf,
                  const string & chan,
                  const slam_state_t * slam_state)
{
  last_yaw = slam_state->yaw;
}

int main() {
  IMUTransformer imuTransformer;
  lcm::LCM lcm;
  lcm.subscribe(PERFECT_IMU_CHANNEL, &IMUTransformer::handleIMU, &imuTransformer);
  lcm.subscribe(SLAM_STATE_CHANNEL, &IMUTransformer::handleSLAMState, &imuTransformer);

  while(1) {
    lcm.handle();
  }
}
