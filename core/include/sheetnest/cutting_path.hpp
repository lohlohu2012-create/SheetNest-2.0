#pragma once
#include "geometry.hpp"
#include "cutting.hpp"
#include <string>
#include <vector>

namespace sheetnest {

enum class CutType { Pierce, Cut, Rapid };

struct ToolMove {
    CutType type{};
    Point from{}, to{};
    double lengthMm{};
    double speedMMin{};
};

struct CuttingContour {
    std::size_t sheetIndex{};
    std::string instanceId;
    std::size_t contourIndex{};
    bool inner{};
    Polygon polygon;
};

struct CuttingOperation {
    std::size_t operation{};
    std::size_t sheetIndex{};
    std::string instanceId;
    std::size_t contourIndex{};
    bool inner{};
    Polygon contour;
    Point start{}, end{};
    double cutLengthMm{};
    double rapidLengthMm{};
    double pierceSeconds{};
    double cuttingSeconds{};
    double rapidSeconds{};
    double totalSeconds{};
};

struct CuttingPath {
    std::vector<ToolMove> moves;
    std::vector<CuttingOperation> operations;
    double totalCutLengthMm{};
    double totalRapidLengthMm{};
    double totalPiercingSeconds{};
    double totalCuttingSeconds{};
    double totalRapidSeconds{};
    double totalSeconds{};
    int pierces{};
};

struct PathOptions {
    double rapidSpeedMMin{120};
    double pierceSeconds{.25};
    bool innerContoursFirst{true};
};

CuttingPath planCuttingPath(
    const std::vector<Polygon>&,
    const CuttingParameters&,
    const PathOptions&
);

CuttingPath planCuttingRoute(
    const std::vector<CuttingContour>&,
    const CuttingParameters&,
    const PathOptions&
);

CuttingEstimate estimateCuttingPath(
    const CuttingPath&,
    const CuttingParameters&,
    const PathOptions&
);

} // namespace sheetnest
