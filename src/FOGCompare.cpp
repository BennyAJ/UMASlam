#include "FOGCompare.hpp"
#include "Constants.hpp"
#include "Utilities.hpp"
#include <cmath>

using namespace std;

void FOGCompare::handleFOG(const lcm::ReceiveBuffer * rbuf,
                    const string & chan,
                    const fog_t * fog_data)
{

  if (chan == FOG_CHANNEL) 
  {
    cout << "FOG CHANNEL: " << fog_data->data << endl;
    last_fog_data = fog_data->data;
  }
  else if (chan == PERFECT_FOG_CHANNEL) 
  {
    cout << "PERFECT FOG CHANNEL: " << fog_data->data << endl;
    cout << "difference is: " << last_fog_data - fog_data->data << endl;
  }
}


int main() {
  FOGCompare FOGCompare;
  lcm::LCM lcm;
  lcm.subscribe(PERFECT_FOG_CHANNEL, &FOGCompare::handleFOG, &FOGCompare);
  lcm.subscribe(FOG_CHANNEL, &FOGCompare::handleFOG, &FOGCompare);

  while(1) {
    lcm.handle();
  }
}
