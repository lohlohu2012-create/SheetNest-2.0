#include "sheetnest/dxf_export.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>

namespace sheetnest {
namespace {

Polygon transform(
    const Polygon& polygon,
    int rotation,
    double x,
    double y
) {
    return translate(rotate(polygon, rotation), x, y);
}

void writePair(
    std::ostringstream& out,
    int code,
    const std::string& value
) {
    out << code << "\n" << value << "\n";
}

void writeNumber(
    std::ostringstream& out,
    int code,
    double value
) {
    out << code << "\n"
        << std::setprecision(15)
        << value << "\n";
}

void writeLayeredPolyline(
    std::ostringstream& out,
    const Polygon& polygon,
    const std::string& layer,
    bool closed
) {
    if (polygon.size() < 2) return;

    writePair(out, 0, "LWPOLYLINE");
    writePair(out, 8, layer);
    writePair(out, 90, std::to_string(polygon.size()));
    writePair(out, 70, closed ? "1" : "0");

    for (const auto& point : polygon) {
        writeNumber(out, 10, point.x);
        writeNumber(out, 20, point.y);
    }
}

void writeText(
    std::ostringstream& out,
    Point point,
    const std::string& layer,
    const std::string& value,
    double height
) {
    writePair(out, 0, "TEXT");
    writePair(out, 8, layer);
    writeNumber(out, 10, point.x);
    writeNumber(out, 20, point.y);
    writeNumber(out, 40, height);
    writePair(out, 1, value);
}

Bounds placedBounds(
    const Instance& instance,
    const Placement& placement
) {
    return bounds(transform(
        instance.part.outer,
        placement.rotation,
        placement.x,
        placement.y
    ));
}

} // namespace

std::string exportNestDxf(
    const Result& result,
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const DxfExportOptions& options
) {
    std::unordered_map<std::string, const Instance*> byId;
    byId.reserve(instances.size());

    for (const auto& instance : instances) {
        byId[instance.id] = &instance;
    }

    std::ostringstream out;
    out << std::fixed << std::setprecision(6);

    writePair(out, 0, "SECTION");
    writePair(out, 2, "HEADER");
    writePair(out, 9, "$INSUNITS");
    writePair(out, 70, "4");
    writePair(out, 0, "ENDSEC");

    writePair(out, 0, "SECTION");
    writePair(out, 2, "ENTITIES");

    const double spacing = std::max(0.0, options.sheetSpacingMm);

    for (std::size_t sheetIndex = 0; sheetIndex < result.sheets.size(); ++sheetIndex) {
        const double sheetOffsetX =
            static_cast<double>(sheetIndex) *
            (sheet.width + spacing);

        const std::string sheetLayer =
            "SHEET_" + std::to_string(sheetIndex + 1);

        if (options.includeSheetOutlines && sheet.width > 0.0 && sheet.height > 0.0) {
            Polygon outline{
                {sheetOffsetX, 0.0},
                {sheetOffsetX + sheet.width, 0.0},
                {sheetOffsetX + sheet.width, sheet.height},
                {sheetOffsetX, sheet.height}
            };
            writeLayeredPolyline(out, outline, sheetLayer, true);
        }

        for (const auto& placement : result.sheets[sheetIndex]) {
            const auto it = byId.find(placement.id);
            if (it == byId.end()) continue;

            const Instance& instance = *it->second;
            const std::string baseLayer =
                instance.part.layer.empty()
                    ? "PARTS"
                    : instance.part.layer;

            const std::string partLayer =
                baseLayer +
                "_S" +
                std::to_string(sheetIndex + 1);

            const double x = placement.x + sheetOffsetX;
            const double y = placement.y;

            const Polygon outer =
                transform(instance.part.outer, placement.rotation, x, y);

            writeLayeredPolyline(out, outer, partLayer, true);

            for (const auto& hole : instance.part.holes) {
                const Polygon transformedHole =
                    transform(hole, placement.rotation, x, y);
                writeLayeredPolyline(
                    out,
                    transformedHole,
                    partLayer + "_HOLE",
                    true
                );
            }

            if (options.includePartIds && !outer.empty()) {
                const auto b = bounds(outer);
                writeText(
                    out,
                    {b.minX, b.maxY},
                    partLayer + "_ID",
                    instance.id,
                    std::max(1.0, std::min(sheet.width, sheet.height) * 0.01)
                );
            }
        }
    }

    writePair(out, 0, "ENDSEC");
    writePair(out, 0, "EOF");

    return out.str();
}

} // namespace sheetnest
