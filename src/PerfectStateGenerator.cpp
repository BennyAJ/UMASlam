#include "PerfectStateGenerator.hpp"
#include "Constants.hpp"
#include "Utilities.hpp"
#include <cmath>

using namespace std;

void PerfectStateGenerator::handleIMU(const lcm::ReceiveBuffer * rbuf,
                    const string & chan,
                    const imu_t * imu_data)
{
  imu_set = true;
  perfect_state.yaw = 2 * M_PI - imu_data->yaw;
}


void PerfectStateGenerator::handleGPS(const lcm::ReceiveBuffer * rbuf,
                  const string & chan,
                  const gps_t * gps_data)
{
  gps_set = true;
  coord_transformer.initialize(perfect_state.lat_origin, perfect_state.lon_origin);
  pair<double, double> coords = coord_transformer.transform(gps_data->latitude, gps_data->longitude);
  cout << "Latitude: " << gps_data->latitude << endl;
  cout << "Longitude: " << gps_data->longitude << endl;
  cout << "X: " << coords.first << endl;
  cout << "Y: " << coords.second << endl;
  perfect_state.x = coords.first;
  perfect_state.y = coords.second;
}

void PerfectStateGenerator::handleSlamState(const lcm::ReceiveBuffer * rbuf,
                  const string & chan,
                  const slam_state_t * slam_state_data)
{
  slam_state_set = true;
  perfect_state.utime = slam_state_data->utime;
  perfect_state.lat_origin = slam_state_data->lat_origin;
  perfect_state.lon_origin = slam_state_data->lon_origin;
}

bool PerfectStateGenerator::readyToPublish() {
  return slam_state_set && imu_set && gps_set;
}

void PerfectStateGenerator::publishPerfectState() {
  slam_state_set = false;
  imu_set = false;
  gps_set = false;

  lcm.publish(PERFECT_STATE_CHANNEL, &perfect_state);
}

int main() {
  lcm::LCM lcm;

  PerfectStateGenerator perfectStateGenerator;
  lcm.subscribe(PERFECT_IMU_CHANNEL, &PerfectStateGenerator::handleIMU, &perfectStateGenerator);
  lcm.subscribe(PERFECT_GPS_CHANNEL, &PerfectStateGenerator::handleGPS, &perfectStateGenerator);
  lcm.subscribe(GPS_STATE_CHANNEL, &PerfectStateGenerator::handleSlamState, &perfectStateGenerator);

  while(true) {
    lcm.handle();

    if(perfectStateGenerator.readyToPublish()) {
      perfectStateGenerator.publishPerfectState();
    }
  }
}
