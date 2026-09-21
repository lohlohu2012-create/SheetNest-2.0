#pragma once
#include "geometry.hpp"
#include <cstddef>
#include <string>
#include <vector>

namespace sheetnest {
struct DxfDiagnostic {
  std::string entityType;
  std::string sourceId;
  std::string message;
  bool warning{};
};
struct DxfContour { Polygon outer; std::vector<Polygon> holes; std::string sourceId; };
struct DxfDocument {
  std::vector<DxfContour> contours;
  std::vector<DxfDiagnostic> diagnostics;
  std::size_t entityCount{};
  std::size_t closedPathCount{};
};
DxfDocument importDxf(const std::string&,double arcToleranceMm=0.25);
DxfDocument importDxfFile(const std::string& path,double arcToleranceMm=0.25);
}
