#include "IMUTransformer.hpp"
#include "Constants.hpp"
#include "Utilities.hpp"
#include <cmath>

using namespace std;

void IMUTransformer::handleIMU(const lcm::ReceiveBuffer * rbuf,
                    const string & chan,
                    const imu_t * imu_data)
{
  /*cout << "imu utime: " << imu_data->utime << endl;
  cout << "udot: " << imu_data->udot << endl;
  cout << "vdot: " << imu_data->vdot << endl;
  cout << "wdot: " << imu_data->wdot << endl;*/
  imu_t publish_imu;
  // copy over most of the IMU, then we'll just change the parts
  // that need to be transformed
  publish_imu = *imu_data;

  // local x acceleration = sideways on the boat = vdot
  // local y acceleration = forward on the boat = udot
  // local z acceleration = vertical on the boat = wdot
  // Roll, Pitch, Yaw = https://en.wikipedia.org/wiki/Aircraft_principal_axes
  // Roll axis is Y, Pitch axis is X, Yaw axis is Z
  // We need to convert these into XYZ in the earth frame, 
  // where Y is north, X is east, Z is down
  // See 3.2.3 in https://arxiv.org/pdf/1704.06053.pdf for explanation on
  // this transformation.

  // Calculating wdot
  double wdot_term = imu_data->wdot * cos(imu_data->roll) * cos(imu_data->yaw);
  double udot_term = imu_data->udot * cos(imu_data->roll) * sin(imu_data->yaw);
  double vdot_term = -1 * imu_data->vdot * sin(imu_data->roll);
  publish_imu.wdot = wdot_term + udot_term + vdot_term;

  // Calculating udot
  wdot_term = sin(imu_data->pitch) * sin(imu_data->roll) * cos(imu_data->yaw);
  wdot_term -= cos(imu_data->pitch) * sin(imu_data->yaw);
  wdot_term *= imu_data->wdot;

  udot_term = sin(imu_data->pitch) * sin(imu_data->roll) * sin(imu_data->yaw);
  udot_term += cos(imu_data->pitch) * cos(imu_data->yaw);
  udot_term *= imu_data->udot;

  vdot_term = imu_data->vdot * sin(imu_data->pitch) * cos(imu_data->roll);
  publish_imu.udot = wdot_term + udot_term + vdot_term;

  // Calculating vdot
  wdot_term = cos(imu_data->pitch) * sin(imu_data->roll) * cos(imu_data->yaw);
  wdot_term += sin(imu_data->pitch) * sin(imu_data->yaw);
  wdot_term *= imu_data->wdot;

  udot_term = cos(imu_data->pitch) * sin(imu_data->roll) * sin(imu_data->yaw);
  udot_term -= sin(imu_data->pitch) * cos(imu_data->yaw);
  udot_term *= imu_data->udot;

  vdot_term = imu_data->vdot * cos(imu_data->pitch) * cos(imu_data->roll);
  publish_imu.vdot = wdot_term + udot_term + vdot_term;

  lcm.publish(TRANSFORMED_IMU_CHANNEL, &publish_imu);
}

void IMUTransformer::handleSimPose(const lcm::ReceiveBuffer * rbuf,
           const std::string & chan,
           const simulator_pose_t * simulator_pose_data)
{
  /*cout << "sim utime: " << simulator_pose_data->utime << endl;
  cout << "sim local x accel: " << simulator_pose_data->local_linear_acceleration_x << endl;
  cout << "sim local y accel: " << simulator_pose_data->local_linear_acceleration_y << endl;
  cout << "sim local z accel: " << simulator_pose_data->local_linear_acceleration_z << endl;*/
}

int main() {
  IMUTransformer imuTransformer;
  lcm::LCM lcm;
  lcm.subscribe(PERFECT_IMU_CHANNEL, &IMUTransformer::handleIMU, &imuTransformer);
  lcm.subscribe(SIM_POSE_CHANNEL, &IMUTransformer::handleSimPose, &imuTransformer);

  while(1) {
    lcm.handle();
  }
}
