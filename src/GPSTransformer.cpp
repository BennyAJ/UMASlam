#include "GPSTransformer.hpp"
#include "Constants.hpp"
#include "Utilities.hpp"
#include <cmath>

using namespace std;
using namespace common::LCM::types;

void GPSTransformer::handleGPS(const lcm::ReceiveBuffer * rbuf,
                    const string & chan,
                    const gps_t * gps_data)
{
  // initializing both coord_transformers
  if (!perfect_gps_transformer.isInitialized()) 
  {
    perfect_gps_transformer.initialize(gps_data->latitude, gps_data->longitude);
  }

  if (!gps_transformer.isInitialized()) 
  {
    gps_transformer.initialize(gps_data->latitude, gps_data->longitude);
  }


  if (chan == PERFECT_GPS_CHANNEL) 
  {
    last_coord_perfect_gps = perfect_gps_transformer.transform(gps_data->latitude, gps_data->longitude);

    cout << "difference in latitude is " << last_coord_perfect_gps.first - last_coord_gps.first << endl;
    cout << "difference in longitude is " << last_coord_perfect_gps.second - last_coord_gps.second << endl;
  }
  else if (chan == GPS_CHANNEL) 
  {
    last_coord_gps = gps_transformer.transform(gps_data->latitude, gps_data->longitude);

    cout << "difference in latitude is " << last_coord_perfect_gps.first - last_coord_gps.first << endl;
    cout << "difference in longitude is " << last_coord_perfect_gps.second - last_coord_gps.second << endl;
  }
}


int main() {
  GPSTransformer gpsTransformer;
  lcm::LCM lcm;
  lcm.subscribe(PERFECT_GPS_CHANNEL, &GPSTransformer::handleGPS, &gpsTransformer);
  lcm.subscribe(GPS_CHANNEL, &GPSTransformer::handleGPS, &gpsTransformer);

  while(1) {
    lcm.handle();
  }
}