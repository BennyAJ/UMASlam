#include "Localizer.hpp"
#include "Constants.hpp"
#include "../lcmtypes/slam_state_t.hpp"
#include <iostream>

using namespace std;

void Localizer::handleGPSData(const lcm::ReceiveBuffer * rbuf,
                const string & chan,
                const gps_t * gps_data)
{
  //if not initialized, initialize the coordinate transformation
  if(!coord_transformer.isInitialized())
  {
    coord_transformer.initialize(gps_data->latitude, gps_data->longitude);
  }
  publishGPSState(gps_data);
}

void Localizer::handleIMUData(const lcm::ReceiveBuffer * rbuf,
           const std::string & chan,
           const imu_t * imu_data)
{
  // Initialize the utime so we don't get an incorrect riemann sum on
  // the first time step. This means we'll lose the very first IMU
  // measurement though, which could be a potential issue
  if(!imu_state_initialized) {
    imu_state.utime = imu_data->utime;
    imu_state_initialized = true;
  }

  // Save information from previous IMU handle 
  last_imu_utime = imu_state.utime;
  last_vel = current_vel;

  imu_state.utime = imu_data->utime;
  imu_state.yaw = imu_data->yaw;

  // Trapezoidal Riemann sum between last two accelerations to find velocity
  double time_diff = (imu_state.utime - last_imu_utime) / 1000000.0;
  current_vel.x += time_diff * ((last_accel.x + imu_data->vdot) / 2);
  current_vel.y += time_diff * ((last_accel.y + imu_data->udot) / 2);

  // Trapezoidal Riemann sum between last two velocities to find position 
  imu_state.x += time_diff * ((last_vel.x + current_vel.x) / 2);
  imu_state.y += time_diff * ((last_vel.y + current_vel.y) / 2);

  imu_state.lat_origin = coord_transformer.getOriginLat();
  imu_state.lon_origin = coord_transformer.getOriginLon();

  last_accel.x = imu_data->vdot;
  last_accel.y = imu_data->udot;

  l.publish(IMU_STATE_CHANNEL, &imu_state);
}

void Localizer::publishGPSState(const gps_t * gps_data) {
  slam_state_t gps_state;

  gps_state.utime = gps_data->utime;
  pair<double, double> last_coord = coord_transformer.transform(gps_data->latitude, gps_data->longitude);
  gps_state.x = last_coord.first;
  gps_state.y = last_coord.second;
  gps_state.yaw = DEG_TO_RAD(gps_data->course_over_ground);
  gps_state.lat_origin = coord_transformer.getOriginLat();
  gps_state.lon_origin = coord_transformer.getOriginLon();
  l.publish(GPS_STATE_CHANNEL, &gps_state);
}
