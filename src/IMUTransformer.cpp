#include "IMUTransformer.hpp"
#include "Constants.hpp"
#include "Utilities.hpp"
#include <cmath>

using namespace std;
void IMUTransformer::handleIMU(const lcm::ReceiveBuffer * rbuf,
                    const string & chan,
                    const imu_t * imu_data)
{
  global_imu_t global_imu;
  global_imu.utime = imu_data->utime;

  global_imu.pitch = imu_data->pitch;
  global_imu.roll = imu_data->roll;
  global_imu.yaw = imu_data->yaw;

  double roll = imu_data->roll;
  // Pitch is counterclockwise in our IMU, but our rotation is simpler
  // if we make it clockwise so that it matches roll and yaw
  double pitch = -1 * imu_data->pitch;
  double yaw = imu_data->yaw;

  // Convert yaw into range -pi to pi
  yaw = -1 * atan2(sin(yaw), cos(yaw));

  // The transform is actually designed to convert global to local, but
  // negating the angles gives the opposite effect
  roll *= -1;
  pitch *= -1;
  yaw *= -1;

  // We use an euler transform to rotate our body frame into a global frame

  // At the end of the calculation, these variables will correspond to
  // global east, north, and down respectively
  double east_accel = imu_data->udot;
  double north_accel = imu_data->vdot;
  double down_accel = imu_data->wdot;

  // | cos(yaw)   sin(yaw)    0 |   | last_east_accel |
  // | -sin(yaw)  cos(yaw)    0 | * | last_north_accel  | 
  // |      0         0       1 |   | last_down_accel  | 

  // We don't want these to change while we're in the middle of multiplying
  // so we need to save their state before we start multiplying
  double last_east_accel = east_accel;
  double last_north_accel = north_accel;
  double last_down_accel = down_accel;

  east_accel = last_east_accel * cos(yaw) + last_north_accel * sin(yaw);
  north_accel = last_east_accel * -1 * sin(yaw) + last_north_accel * cos(yaw);
  down_accel = last_down_accel;

  // | cos(pitch)   0   -sin(pitch) |   | last_east_accel |
  // |     0        1        0      | * | last_north_accel  |
  // | sin(pitch)   0    cos(pitch) |   | last_down_accel  |

  last_east_accel = east_accel;
  last_north_accel = north_accel;
  last_down_accel = down_accel;

  east_accel = last_east_accel * cos(pitch) + last_down_accel * -1 * sin(pitch);
  north_accel = last_north_accel;
  down_accel = last_east_accel * sin(pitch) + last_down_accel * cos(pitch);

  // | 1        0       0         |   | last_east_accel |
  // | 0    cos(roll)   sin(roll) | * | last_north_accel  |
  // | 0    -sin(roll)  cos(roll) |   | last_down_accel  |
  
  last_east_accel = east_accel;
  last_north_accel = north_accel;
  last_down_accel = down_accel;

  east_accel = last_east_accel;
  north_accel = last_north_accel * cos(roll) + last_down_accel * sin(roll);
  down_accel = last_north_accel * -1 * sin(roll) + last_down_accel * cos(roll);

  // Remove gravity from the down axis
  down_accel -= 9.80655;

  global_imu.east_accel = east_accel;
  global_imu.north_accel = north_accel;
  global_imu.down_accel = down_accel;

  lcm.publish(TRANSFORMED_IMU_CHANNEL, &global_imu);
}

int main() {
  IMUTransformer imuTransformer;
  lcm::LCM lcm;
  lcm.subscribe(IMU_CHANNEL, &IMUTransformer::handleIMU, &imuTransformer);

  while(1) {
    lcm.handle();
  }
}
