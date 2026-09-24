#include "sheetnest/pipeline_validator.hpp"

#include "sheetnest/cutting_path.hpp"
#include "sheetnest/dxf_model.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace sheetnest {
namespace {

void addStage(
    ProductionPipelineReport& report,
    const char* name,
    PipelineStatus status,
    const std::string& reason = {},
    std::size_t a = 0,
    std::size_t b = 0,
    double value = 0.0
) {
    report.stages.push_back({name, status, reason, a, b, value});
    if (status == PipelineStatus::Fail && report.failedStage.empty()) {
        report.failedStage = name;
        report.failureReason = reason;
    }
}

bool finitePolygon(const Polygon& polygon) {
    for (const auto& p : polygon) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) return false;
    }
    return polygon.size() >= 3;
}

std::vector<Polygon> placedContours(
    const Result& result,
    const std::vector<Instance>& instances
) {
    std::unordered_map<std::string, const Instance*> byId;
    for (const auto& i : instances) byId.emplace(i.id, &i);

    std::vector<Polygon> contours;
    for (const auto& sheetPlacements : result.sheets) {
        for (const auto& placement : sheetPlacements) {
            const auto it = byId.find(placement.id);
            if (it == byId.end()) continue;

            const auto outer = translate(
                rotate(it->second->part.outer, placement.rotation),
                placement.x,
                placement.y
            );
            if (finitePolygon(outer)) contours.push_back(outer);

            for (const auto& hole : it->second->part.holes) {
                const auto transformed = translate(
                    rotate(hole, placement.rotation),
                    placement.x,
                    placement.y
                );
                if (finitePolygon(transformed)) contours.push_back(transformed);
            }
        }
    }
    return contours;
}

} // namespace

ProductionPipelineReport validateProductionPipeline(
    const std::string& sourceDxf,
    const Sheet& sheet,
    const Options& options,
    const CuttingParameters& technology,
    const DxfExportOptions& exportOptions
) {
    ProductionPipelineReport report;

    const auto document = importDxf(sourceDxf, 0.25);
    if (document.hasErrors() || document.contours.empty()) {
        addStage(
            report,
            "DXF Parse",
            PipelineStatus::Fail,
            document.contours.empty()
                ? "DXF не содержит замкнутых контуров."
                : "DXF содержит ошибки парсинга.",
            document.entitiesRead,
            document.supportedEntities
        );
        return report;
    }
    addStage(
        report,
        "DXF Parse",
        PipelineStatus::Ok,
        {},
        document.entitiesRead,
        document.supportedEntities
    );

    if (!document.valid()) {
        addStage(
            report,
            "DXF Preflight",
            PipelineStatus::Fail,
            "Preflight отклонил один или несколько контуров."
        );
        return report;
    }
    addStage(
        report,
        "DXF Preflight",
        PipelineStatus::Ok,
        {},
        document.closedLoopsFound,
        document.contours.size()
    );

    const auto instances = instancesFromDxf(document, 1);
    report.expectedInstances = instances.size();
    const auto result = nest(instances, sheet, options);

    report.placedInstances =
        instances.size() - result.unplaced.size();
    report.unplacedInstances = result.unplaced.size();
    report.sheets = result.sheets.size();

    if (!result.unplaced.empty()) {
        addStage(
            report,
            "Nesting",
            PipelineStatus::Fail,
            "Не все экземпляры получили допустимое размещение.",
            report.placedInstances,
            report.expectedInstances,
            result.utilization
        );
        return report;
    }
    addStage(
        report,
        "Nesting",
        PipelineStatus::Ok,
        {},
        report.placedInstances,
        report.expectedInstances,
        result.utilization
    );

    // No repair is required when nesting already placed every requested
    // instance. Keep the stage explicit so the production report distinguishes
    // "repair not required" from Coverage validation.
    addStage(
        report,
        "Adaptive Repair",
        PipelineStatus::Skipped,
        "Не требуется: все экземпляры размещены после Nesting."
    );

    const auto contours = placedContours(result, instances);
    bool coverageOk = contours.size() >= report.placedInstances;
    if (!coverageOk) {
        addStage(
            report,
            "Coverage",
            PipelineStatus::Fail,
            "Для части размещённых деталей не сформирована геометрия покрытия."
        );
        return report;
    }
    addStage(
        report,
        "Coverage",
        PipelineStatus::Ok,
        {},
        contours.size(),
        report.placedInstances
    );

    const auto path = planCuttingPath(
        contours,
        technology,
        PathOptions{}
    );

    if (path.moves.empty()) {
        addStage(
            report,
            "CAM Route",
            PipelineStatus::Fail,
            "CAM route не сформирован."
        );
        return report;
    }

    addStage(
        report,
        "CAM Route",
        PipelineStatus::Ok,
        {},
        path.moves.size(),
        static_cast<std::size_t>(path.pierces),
        path.totalCutLengthMm
    );

    bool camOk = !path.moves.empty() &&
                 path.totalCutLengthMm > 0.0 &&
                 path.pierces > 0;
    if (!camOk) {
        addStage(
            report,
            "CAM Validation",
            PipelineStatus::Fail,
            "CAM route пуст или не содержит валидных операций."
        );
        return report;
    }

    addStage(
        report,
        "CAM Validation",
        PipelineStatus::Ok,
        {},
        path.moves.size(),
        static_cast<std::size_t>(path.pierces),
        path.totalCutLengthMm
    );
    report.camOperations = path.moves.size();

    const auto exported = exportNestDxf(
        result,
        instances,
        sheet,
        exportOptions
    );
    report.exportedDxfBytes = exported.size();

    if (exported.empty()) {
        addStage(
            report,
            "DXF Export",
            PipelineStatus::Fail,
            "Экспорт DXF вернул пустой документ."
        );
        return report;
    }
    addStage(
        report,
        "DXF Export",
        PipelineStatus::Ok,
        {},
        exported.size()
    );

    const auto roundTrip = importDxf(exported, 0.25);
    report.roundTripValid =
        !roundTrip.hasErrors() &&
        !roundTrip.contours.empty() &&
        roundTrip.closedLoopsFound >= report.placedInstances;

    if (!report.roundTripValid) {
        addStage(
            report,
            "DXF Round-trip",
            PipelineStatus::Fail,
            "Повторный импорт экспортированного DXF не прошёл проверку."
        );
        return report;
    }

    addStage(
        report,
        "DXF Round-trip",
        PipelineStatus::Ok,
        {},
        roundTrip.closedLoopsFound,
        report.placedInstances
    );

    addStage(
        report,
        "Complete",
        PipelineStatus::Ok,
        "Все производственные этапы прошли проверку."
    );
    report.complete = true;
    return report;
}

const char* toString(PipelineStatus status) {
    switch (status) {
    case PipelineStatus::Pending: return "—";
    case PipelineStatus::Ok: return "OK";
    case PipelineStatus::Fail: return "FAIL";
    case PipelineStatus::Skipped: return "—";
    }
    return "—";
}

} // namespace sheetnest
