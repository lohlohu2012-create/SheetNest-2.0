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
    std::size_t entityIndex{};
    std::string entityType;
    std::string layer;
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
    std::size_t supportedEntities{};
    std::size_t unsupportedEntities{};
    std::size_t malformedEntities{};
    std::size_t closedLoopsFound{};

    bool valid() const;
    bool hasErrors() const;
    bool hasWarnings() const;
};

DxfDocument importDxf(const std::string& text, double arcToleranceMm = 0.25);

} // namespace sheetnest
