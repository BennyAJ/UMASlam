#include "IMUTransformer.hpp"
#include "Constants.hpp"
#include "Utilities.hpp"
#include <cmath>

using namespace std;
void IMUTransformer::handleIMU(const lcm::ReceiveBuffer * rbuf,
                    const string & chan,
                    const imu_t * imu_data)
{
  imu_t publish_imu;
  publish_imu = *imu_data;

  double roll = imu_data->roll;
  // Pitch is counterclockwise in our IMU, but our rotation is simpler
  // if we make it clockwise so that it matches roll and yaw
  double pitch = -1 * imu_data->pitch;
  double yaw = imu_data->yaw;

  // Convert yaw into range -pi to pi
  yaw = -1 * atan2(sin(yaw), cos(yaw));

  publish_imu.roll = roll;
  publish_imu.pitch = pitch;
  publish_imu.yaw = yaw;

  // The transform is actually designed to convert global to local, but
  // negating the angles gives the opposite effect
  roll *= -1;
  pitch *= -1;
  yaw *= -1;

  // We use an euler transform to rotate our body frame into a global frame

  // At the end of the calculation, these variables will correspond to
  // global north, east, and down respectively
  double north_accel = imu_data->udot;
  double east_accel = imu_data->vdot;
  double down_accel = imu_data->wdot;

  // | cos(yaw)   sin(yaw)    0 |   | last_north_accel |
  // | -sin(yaw)  cos(yaw)    0 | * | last_east_accel  | 
  // |      0         0       1 |   | last_down_accel  | 

  // We don't want these to change while we're in the middle of multiplying
  // so we need to save their state before we start multiplying
  double last_north_accel = north_accel;
  double last_east_accel = east_accel;
  double last_down_accel = down_accel;

  north_accel = last_north_accel * cos(yaw) + last_east_accel * sin(yaw);
  east_accel = last_north_accel * -1 * sin(yaw) + last_east_accel * cos(yaw);
  down_accel = last_down_accel;

  // | cos(pitch)   0   -sin(pitch) |   | last_north_accel |
  // |     0        1        0      | * | last_east_accel  |
  // | sin(pitch)   0    cos(pitch) |   | last_down_accel  |

  last_north_accel = north_accel;
  last_east_accel = east_accel;
  last_down_accel = down_accel;

  north_accel = last_north_accel * cos(pitch) + last_down_accel * -1 * sin(pitch);
  east_accel = last_east_accel;
  down_accel = last_north_accel * sin(pitch) + last_down_accel * cos(pitch);

  // | 1        0       0         |   | last_north_accel |
  // | 0    cos(roll)   sin(roll) | * | last_east_accel  |
  // | 0    -sin(roll)  cos(roll) |   | last_down_accel  |
  
  last_north_accel = north_accel;
  last_east_accel = east_accel;
  last_down_accel = down_accel;

  north_accel = last_north_accel;
  east_accel = last_east_accel * cos(roll) + last_down_accel * sin(roll);
  down_accel = last_east_accel * -1 * sin(roll) + last_down_accel * cos(roll);

  // Remove gravity from the down axis
  down_accel -= 9.80655;

  publish_imu.udot = north_accel;
  publish_imu.vdot = east_accel;
  publish_imu.wdot = down_accel;

  lcm.publish(TRANSFORMED_IMU_CHANNEL, &publish_imu);
}

void IMUTransformer::handleSimPose(const lcm::ReceiveBuffer * rbuf,
           const std::string & chan,
           const simulator_pose_t * simulator_pose_data)
{
  /*
  // Use this to verify that the algorithm is accurate with perfect data
  
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
  */
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
