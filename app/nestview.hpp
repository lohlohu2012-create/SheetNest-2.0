#pragma once

#include <QGraphicsView>
#include "sheetnest/nesting.hpp"
#include "sheetnest/production_validation.hpp"
#include "sheetnest/cutting_path.hpp"

class NestView final : public QGraphicsView {
public:
    using CuttingRouteOperation = sheetnest::CuttingOperation;

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
    void setCuttingRoute(const sheetnest::CuttingPath& route, double sheetHeight);
    void setCuttingRouteVisible(bool visible);
    void setCuttingAnimationProgress(double progress);
    void setCuttingAnimationOperation(int operation);
    void setCuttingAnimationOperationProgress(int operation, double progress);

    const std::vector<CuttingRouteOperation>& cuttingRouteOperations() const {
        return cuttingRoute_.operations;
    }

protected:
    void wheelEvent(QWheelEvent* event) override;

private:
    bool cuttingRouteVisible_{false};
    double cuttingAnimationProgress_{1.0};
    int cuttingAnimationOperation_{-1};
    double cuttingAnimationOperationProgress_{0.0};
    sheetnest::CuttingPath cuttingRoute_;
    double sheetHeight_{0.0};

    void addCuttingRoute();
    QGraphicsPathItem* addPartItem(
        QGraphicsItem* parent,
        const sheetnest::Instance& instance,
        const sheetnest::Placement& placement,
        std::size_t colorIndex
    );
};
