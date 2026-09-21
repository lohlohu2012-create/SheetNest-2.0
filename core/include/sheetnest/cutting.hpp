#pragma once
#include <string>
namespace sheetnest {
struct CuttingParameters {
  double laserPowerKw{3.0};
  std::string material;
  double thicknessMm{};
  std::string gas;
  double speedMMin{};
  double pierceSeconds{0.25};
};
struct CuttingEstimate {
  CuttingParameters parameters;
  double contourLengthMm{};
  double cuttingMinutes{};
  double piercingMinutes{};
  double rapidMinutes{};
  double totalMinutes{};
  int pierces{};
};
}
