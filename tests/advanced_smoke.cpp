#include "sheetnest/nfp_search.hpp"
#include "sheetnest/pipeline_validator.hpp"
#include "sheetnest/watchdog.hpp"

#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>

using namespace sheetnest;

namespace {

const char* kRectangleDxf = R"DXF(
0
SECTION
2
ENTITIES
0
LWPOLYLINE
8
PART
70
1
10
0
20
0
10
100
20
0
10
100
20
80
10
0
20
80
ENDSEC
0
EOF
)DXF";

void testNfpSearchBudget() {
    Polygon fixed{{0,0},{100,0},{100,20},{60,20},{60,70},{0,70}};
    Polygon moving{{0,0},{20,0},{20,15},{0,15}};

    const auto region = nfp::feasibilityRegion(
        fixed, moving, 0, -20, -20, 120, 100, 2.0
    );
    assert(!region.boundary.empty());

    nfp::SearchOptions options;
    options.budgetMs = 1000.0;
    options.segmentSpacingMm = 2.0;
    options.maxCandidates = 32;
    options.includeSheetBoundary = false;

    const auto result = nfp::searchFeasibilityRegion(region, options);
    assert(!result.candidates.empty());
    assert(result.candidates.size() <= 32);
    assert(result.telemetry.candidateChecks >= result.candidates.size());
    assert(result.telemetry.segmentsSampled > 0);
    assert(result.telemetry.elapsedMs >= 0.0);
}

void testWatchdogTransitions() {
    Watchdog watchdog(20.0, 1000.0);
    watchdog.stage(WatchdogStage::NfpSearch, "test");
    watchdog.heartbeat(1, 10, "alive");
    auto running = watchdog.poll();
    assert(running.state == WatchdogState::Running);
    assert(running.stage == WatchdogStage::NfpSearch);

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    const auto timeout = watchdog.poll();
    assert(timeout.state == WatchdogState::Timeout);
    assert(watchdog.stopRequested());
}

void testPipelineValidator() {
    Sheet sheet{200, 200, 5.0};
    Options options;
    options.rotations = {0, 90};
    options.iterations = 2;
    options.gapMm = 2.0;
    options.overallBudgetMs = 5000.0;
    options.nfpSearchBudgetMs = 100.0;

    const auto technology =
        bodor3kWParameters(Material::CarbonSteel, 3.0);

    const auto report = validateProductionPipeline(
        kRectangleDxf,
        sheet,
        options,
        technology
    );

    assert(report.complete);
    assert(report.failedStage.empty());
    assert(report.expectedInstances == 1);
    assert(report.placedInstances == 1);
    assert(report.unplacedInstances == 0);
    assert(report.sheets == 1);
    assert(report.camOperations > 0);
    assert(report.exportedDxfBytes > 0);
    assert(report.roundTripValid);
    assert(!report.stages.empty());
}

} // namespace

int main() {
    testNfpSearchBudget();
    testWatchdogTransitions();
    testPipelineValidator();
    std::cout << "SheetNest advanced smoke tests passed\n";
    return 0;
}
