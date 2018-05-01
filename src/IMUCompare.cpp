#include "IMUCompare.hpp"
#include "Constants.hpp"
#include "Utilities.hpp"
#include <cmath>

using namespace std;

void IMUCompare::handleIMU(const lcm::ReceiveBuffer * rbuf,
                    const string & chan,
                    const imu_t * imu_data)
{
  if (chan == PERFECT_IMU_CHANNEL) 
  {
    last_perfect_imu_info = {imu_data->udot, imu_data->vdot};
  }
  else if (chan == IMU_CHANNEL) 
  {
    cout << "difference in udot is: " << last_perfect_imu_info.first - imu_data->udot << endl;
    cout << "difference in vdot is: " << last_perfect_imu_info.second - imu_data->vdot << endl;
  }
}


int main() {
  IMUCompare IMUCompare;
  lcm::LCM lcm;
  lcm.subscribe(PERFECT_IMU_CHANNEL, &IMUCompare::handleIMU, &IMUCompare);
  lcm.subscribe(IMU_CHANNEL, &IMUCompare::handleIMU, &IMUCompare);

  while(1) {
    lcm.handle();
  }
}
