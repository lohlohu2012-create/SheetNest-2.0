#pragma once

#include <QGraphicsView>

#include "sheetnest/nesting.hpp"
#include "sheetnest/production_validation.hpp"\n#include "sheetnest/cutting.hpp"

class NestView final : public QGraphicsView {
public:
    explicit NestView(QWidget* parent = nullptr);

    void showResult(
        const sheetnest::Result& result,
        const std::vector<sheetnest::Instance>& instances,
        const sheetnest::Sheet& sheet,
        const sheetnest::ProductionValidationReport* repairVisualization = nullptr,
        int repairRound = -1,
        bool showConflict = true,
        bool showExtracted = true,
        bool showMoved = true,
        bool showStationary = true,
        int animationStage = -1
    );

    void clearResult();\n\n    void setCuttingRouteVisible(bool visible);

protected:
    void wheelEvent(QWheelEvent* event) override;

private:
    void addCuttingRoute(\n        const sheetnest::Result& result,\n        const std::vector<sheetnest::Instance>& instances,\n        const sheetnest::Sheet& sheet\n    );\n\n    QGraphicsPathItem* addPartItem(
        QGraphicsItem* parent,
        const sheetnest::Instance& instance,
        const sheetnest::Placement& placement,
        std::size_t colorIndex
    );
};
