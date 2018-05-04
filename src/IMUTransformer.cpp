#include "IMUTransformer.hpp"
#include "Constants.hpp"
#include "Utilities.hpp"
#include <cmath>

using namespace std;

void IMUTransformer::handleIMU(const lcm::ReceiveBuffer * rbuf,
                    const string & chan,
                    const imu_t * imu_data)
{
}

void IMUTransformer::handleSimPose(const lcm::ReceiveBuffer * rbuf,
           const std::string & chan,
           const simulator_pose_t * simulator_pose_data)
{
  imu_t publish_imu;

  // local x acceleration = sideways on the boat = vdot
  // local y acceleration = forward on the boat = udot
  // local z acceleration = vertical on the boat = wdot
  // Roll, Pitch, Yaw = https://en.wikipedia.org/wiki/Aircraft_principal_axes
  // Roll axis is Y, Pitch axis is X, Yaw axis is Z
  // We need to convert these into XYZ in the earth frame, 
  // where Y is north, X is east, Z is down
  // See http://mathworld.wolfram.com/EulerAngles.html x y z (pitch-roll-yaw)
  // this transformation.
  // We negate ours because we need to rotate clockwise

  double roll = -1 * simulator_pose_data->roll;
  double pitch = -1 * simulator_pose_data->pitch;
  double yaw = -1 * simulator_pose_data->yaw;
  double local_x_accel = simulator_pose_data->local_linear_acceleration_x;
  double local_y_accel = simulator_pose_data->local_linear_acceleration_y;
  double local_z_accel = simulator_pose_data->local_linear_acceleration_z;

  // Calculating vdot
  double vdot_term = local_x_accel * cos(pitch) * cos(yaw);
  double udot_term = local_y_accel * cos(pitch) * sin(yaw);
  double wdot_term = -1 * local_z_accel * sin(pitch);
  publish_imu.vdot = wdot_term + udot_term + vdot_term;

  // Calculating udot
  wdot_term = sin(roll) * sin(pitch) * cos(yaw);
  wdot_term -= cos(roll) * sin(yaw);
  wdot_term *= local_x_accel;

  udot_term = sin(roll) * sin(pitch) * sin(yaw);
  udot_term += cos(roll) * cos(yaw);
  udot_term *= local_y_accel;

  vdot_term = local_z_accel * cos(pitch) * sin(roll);
  publish_imu.udot = wdot_term + udot_term + vdot_term;

  // Calculating vdot
  wdot_term = cos(roll) * sin(pitch) * cos(yaw);
  wdot_term += sin(roll) * sin(yaw);
  wdot_term *= local_x_accel;

  udot_term = cos(roll) * sin(pitch) * sin(yaw);
  udot_term -= sin(roll) * cos(yaw);
  udot_term *= local_y_accel;

  vdot_term = local_z_accel * cos(pitch) * cos(roll);
  publish_imu.wdot = wdot_term + udot_term + vdot_term;

  lcm.publish(TRANSFORMED_IMU_CHANNEL, &publish_imu);
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
