#pragma once

#include "cutting_path.hpp"

#include <string>

namespace sheetnest {

struct CamExportOptions {
    std::string programName{"SHEETNEST"};
    bool absoluteCoordinates{true};
    bool includeComments{true};
    bool includeSheetMarkers{true};
    bool emitSpindleCommands{true};
    double piercePowerPercent{100.0};
    double cutPowerPercent{100.0};
    double rapidHeightMm{0.0};
};

struct CamValidationReport {
    bool valid{true};
    std::size_t operationCount{};
    std::size_t invalidGeometryCount{};
    std::size_t nonFiniteCoordinateCount{};
    std::size_t nonFiniteTimeCount{};
    std::size_t zeroLengthCutCount{};
    std::string message;
};

CamValidationReport validateCuttingPath(const CuttingPath& path);

std::string exportCamProgram(
    const CuttingPath& path,
    const CuttingParameters& parameters,
    const CamExportOptions& options = {}
);

} // namespace sheetnest
