#pragma once

#include <QGraphicsView>

#include "sheetnest/nesting.hpp"
#include "sheetnest/production_validation.hpp"
#include "sheetnest/cutting.hpp"

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

    void clearResult();

    void setCuttingRouteVisible(bool visible);
    void setCuttingAnimationProgress(double progress);

protected:
    void wheelEvent(QWheelEvent* event) override;

private:
    bool cuttingRouteVisible_{false};
    double cuttingAnimationProgress_{1.0};

    void addCuttingRoute(
        const sheetnest::Result& result,
        const std::vector<sheetnest::Instance>& instances,
        const sheetnest::Sheet& sheet
    );

    QGraphicsPathItem* addPartItem(
        QGraphicsItem* parent,
        const sheetnest::Instance& instance,
        const sheetnest::Placement& placement,
        std::size_t colorIndex
    );
};
