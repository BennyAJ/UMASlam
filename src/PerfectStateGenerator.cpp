#include "PerfectStateGenerator.hpp"
#include "Constants.hpp"
#include "Utilities.hpp"
#include <cmath>

using namespace std;

void PerfectStateGenerator::handleFOG(const lcm::ReceiveBuffer * rbuf,
                    const string & chan,
                    const fog_t * fog_data)
{
  fog_set = true;
  perfect_state.yaw = DEG_TO_RAD(fog_data->data) - perfect_state.north_angle;
}


void PerfectStateGenerator::handleGPS(const lcm::ReceiveBuffer * rbuf,
                  const string & chan,
                  const gps_t * gps_data)
{
  gps_set = true;
  coord_transformer.initialize(perfect_state.lat_origin, perfect_state.lon_origin);
  pair<double, double> coords = coord_transformer.transform(gps_data->latitude, gps_data->longitude);
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
  return slam_state_set && fog_set && gps_set;
}

void PerfectStateGenerator::publishPerfectState() {
  slam_state_set = false;
  fog_set = false;
  gps_set = false;

  lcm.publish(PERFECT_SLAM_STATE_CHANNEL, &perfect_state);
}

int main() {
  lcm::LCM lcm;

  PerfectStateGenerator perfectStateGenerator;
  lcm.subscribe(PERFECT_FOG_CHANNEL, &PerfectStateGenerator::handleFOG, &perfectStateGenerator);
  lcm.subscribe(PERFECT_GPS_CHANNEL, &PerfectStateGenerator::handleGPS, &perfectStateGenerator);
  lcm.subscribe(SLAM_STATE_CHANNEL, &PerfectStateGenerator::handleSlamState, &perfectStateGenerator);

  while(true) {
    lcm.handle();

    if(perfectStateGenerator.readyToPublish()) {
      perfectStateGenerator.publishPerfectState();
    }
  }
}
