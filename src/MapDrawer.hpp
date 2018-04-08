#ifndef __UMA_MAP_DRAWER_HPP__
#define __UMA_MAP_DRAWER_HPP__
#include <unordered_map>
#include "GridMap.hpp"
#include "Constants.hpp"
#include "../lcmtypes/state_t.hpp"
#include "../lcmtypes/slam_map_t.hpp"
#include <SFML/Graphics.hpp>
#include <lcm/lcm-cpp.hpp>
#include <mutex>
#include <vector>
#include <utility>
#include "Pose.hpp"

constexpr double WINDOW_HEIGHT = 1600.0;
constexpr double WINDOW_WIDTH = 1200.0;

constexpr size_t PIX_PER_SQUARE = 4;

class MapDrawer
{
public:
  MapDrawer() {
    possibleChans.push_back(std::pair<std::string,sf::Color> (SLAM_STATE_CHANNEL,sf::Color::Red));
    possibleChans.push_back(std::pair<std::string,sf::Color> (GPS_STATE_CHANNEL,sf::Color::Blue));
  }
  void startDrawThread();
  void switchMap(const GridMap& nmap);
  void addPose(const SLAM::Pose& pose, std::string channel);
  void startDraw();
  void drawMap(sf::RenderWindow & win);
  void drawPoses(sf::RenderWindow & win);
  void drawBoat(sf::RenderWindow & win);

  void handleState(const lcm::ReceiveBuffer * rbuf, const std::string & chan, const common::LCM::types::state_t * state);
  void handleMap(const lcm::ReceiveBuffer * rbuf, const std::string & chan, const SLAM::LCM::slam_map_t * slam_map);

private:
  std::pair<double, double> convertToPixelCoords(double x, double y);

  std::mutex map_mut;
  GridMap map;
  std::unordered_map<std::string,std::vector<SLAM::Pose>> unOrdMap;
  std::vector<std::pair<std::string,sf::Color>>possibleChans;
};
#endif
