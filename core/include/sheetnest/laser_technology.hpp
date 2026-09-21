#pragma once
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace sheetnest {

struct LaserTechnologyPoint {
  double laserPowerKw{3.0};
  std::string material;
  double thicknessMm{};
  std::string gas;
  double speedMinMMin{};
  double speedMaxMMin{};
  double speedMMin{};
  double pierceSeconds{0.25};
  std::string source;
  std::string note;
  bool interpolated{};
};

class LaserTechnologyDatabase {
public:
  bool loadCsv(const std::string& path);
  bool saveCsv(const std::string& path) const;
  void setPoints(std::vector<LaserTechnologyPoint> points);
  const std::vector<LaserTechnologyPoint>& points() const { return points_; }

  std::optional<LaserTechnologyPoint> lookup(
      const std::string& material,
      double thicknessMm,
      const std::string& gas,
      double laserPowerKw = 3.0) const;

private:
  std::vector<LaserTechnologyPoint> points_;
};

}
