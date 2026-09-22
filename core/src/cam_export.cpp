#include "sheetnest/cam_export.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace sheetnest {
namespace {

bool finitePoint(Point p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

void writeXY(
    std::ostringstream& out,
    Point p,
    bool absolute
) {
    if (!absolute) return;
    out << " X" << std::fixed << std::setprecision(3) << p.x
        << " Y" << std::fixed << std::setprecision(3) << p.y;
}

} // namespace

CamValidationReport validateCuttingPath(const CuttingPath& path) {
    CamValidationReport report;
    report.operationCount = path.operations.size();

    for (const auto& op : path.operations) {
        if (!finitePoint(op.rapidFrom) ||
            !finitePoint(op.start) ||
            !finitePoint(op.end)) {
            ++report.nonFiniteCoordinateCount;
        }

        if (!std::isfinite(op.cutLengthMm) ||
            !std::isfinite(op.rapidLengthMm) ||
            !std::isfinite(op.pierceSeconds) ||
            !std::isfinite(op.cuttingSeconds) ||
            !std::isfinite(op.rapidSeconds) ||
            !std::isfinite(op.totalSeconds)) {
            ++report.nonFiniteTimeCount;
        }

        if (op.contour.size() < 3) {
            ++report.invalidGeometryCount;
        } else if (op.cutLengthMm <= 1e-9) {
            ++report.zeroLengthCutCount;
        }
    }

    report.valid =
        report.invalidGeometryCount == 0 &&
        report.nonFiniteCoordinateCount == 0 &&
        report.nonFiniteTimeCount == 0 &&
        report.zeroLengthCutCount == 0;

    report.message =
        report.valid
            ? "CAM route validation: PASS"
            : "CAM route validation: FAIL";
    return report;
}

std::string exportCamProgram(
    const CuttingPath& path,
    const CuttingParameters& parameters,
    const CamExportOptions& options
) {
    const auto validation = validateCuttingPath(path);
    if (!validation.valid) return {};

    std::ostringstream out;
    out << std::fixed << std::setprecision(3);

    if (options.includeComments) {
        out << "; " << options.programName << "\n";
        out << "; SHEETNEST CAM ROUTE\n";
        out << "; Operations: " << path.operations.size() << "\n";
        out << "; Cut length: " << path.totalCutLengthMm << " mm\n";
        out << "; Rapid length: " << path.totalRapidLengthMm << " mm\n";
        out << "; Estimated time: " << path.totalSeconds << " s\n";
        out << "; Material thickness: " << parameters.thicknessMm << " mm\n";
        out << "; Speed: " << parameters.speedMMin << " m/min\n";
        out << "G90\n";
    }

    std::size_t activeSheet = static_cast<std::size_t>(-1);
    for (const auto& op : path.operations) {
        if (options.includeSheetMarkers && op.sheetIndex != activeSheet) {
            activeSheet = op.sheetIndex;
            out << "\n; SHEET " << (activeSheet + 1) << "\n";
        }

        if (options.includeComments) {
            out << "; OP " << (op.operation + 1)
                << " INSTANCE " << op.instanceId
                << " CONTOUR " << op.contourIndex
                << (op.inner ? " INNER\n" : " OUTER\n");
        }

        out << "G0";
        writeXY(out, op.rapidFrom, options.absoluteCoordinates);
        out << "\n";

        out << "G0";
        writeXY(out, op.start, options.absoluteCoordinates);
        out << "\n";

        if (options.emitSpindleCommands) {
            out << "M3 S" << std::clamp(options.piercePowerPercent, 0.0, 100.0)
                << "\n";
        }

        out << "; PIERCE " << op.pierceSeconds << " s\n";
        out << "G4 P" << std::max(0.0, op.pierceSeconds) << "\n";

        if (options.emitSpindleCommands) {
            out << "S" << std::clamp(options.cutPowerPercent, 0.0, 100.0)
                << "\n";
        }

        out << "G1 F" << std::max(0.0, parameters.speedMMin * 1000.0)
            << "\n";

        if (op.contour.size() >= 2) {
            for (std::size_t i = 0; i < op.contour.size(); ++i) {
                out << "G1";
                writeXY(
                    out,
                    op.contour[(i + 1) % op.contour.size()],
                    options.absoluteCoordinates
                );
                out << "\n";
            }
        }

        if (options.emitSpindleCommands) {
            out << "M5\n";
        }
    }

    out << "G0 X0.000 Y0.000\n";
    out << "M2\n";
    return out.str();
}

} // namespace sheetnest
