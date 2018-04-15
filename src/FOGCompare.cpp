#include "FOGCompare.hpp"
#include "Constants.hpp"
#include "Utilities.hpp"
#include <cmath>

using namespace std;
using namespace common::LCM::types;

void FOGCompare::handleFOG(const lcm::ReceiveBuffer * rbuf,
                    const string & chan,
                    const fog_t * fog_data)
{

  if (chan == PERFECT_FOG_CHANNEL) 
  {
    last_perfect_fog_data = fog_data->data;

    cout << "difference in data is 1: " << last_perfect_fog_data - last_fog_data << endl;
    cout << "difference in data is 2: " << last_perfect_fog_data - last_fog_data << endl;
  }
  else if (chan == FOG_CHANNEL) 
  {

    last_fog_data = fog_data->data;

    cout << "difference in data is 3: " << last_perfect_fog_data - last_fog_data << endl;
    cout << "difference in data is 4: " << last_perfect_fog_data - last_fog_data << endl;
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