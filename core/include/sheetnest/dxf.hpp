#pragma once
#include "geometry.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace sheetnest {

enum class DxfSeverity {
    Info,
    Warning,
    Error
};

struct DxfDiagnostic {
    DxfSeverity severity{DxfSeverity::Info};
    std::string stage;
    std::string message;
};

struct DxfContour {
    Polygon outer;
    std::vector<Polygon> holes;
    std::string sourceId;
    std::string layer;
};

struct DxfDocument {
    std::vector<DxfContour> contours;
    std::vector<DxfDiagnostic> diagnostics;
    std::size_t entitiesRead{};
    std::size_t closedLoopsFound{};

    bool valid() const;
};

DxfDocument importDxf(const std::string& text, double arcToleranceMm = 0.25);

} // namespace sheetnest
