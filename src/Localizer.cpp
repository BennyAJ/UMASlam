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
  
  last_utime = gps_data->utime;
  last_coord = coord_transformer.transform(gps_data->latitude, gps_data->longitude);
  last_yaw = DEG_TO_RAD(gps_data->course_over_ground);

  publishPose();
}

void Localizer::publishPose() const
{
  lcm::LCM l;
  slam_state_t pub_state;
  pub_state.utime = last_utime;
  pub_state.x = last_coord.first;
  pub_state.y = last_coord.second;
  pub_state.yaw = last_yaw;
  pub_state.lat_origin = coord_transformer.getOriginLat();
  pub_state.lon_origin = coord_transformer.getOriginLon();
  l.publish(SLAM_STATE_CHANNEL, &pub_state);
}
