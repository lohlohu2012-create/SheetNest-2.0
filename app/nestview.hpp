#pragma once

#include <QGraphicsView>

#include "sheetnest/nesting.hpp"
#include "sheetnest/production_validation.hpp"
#include "sheetnest/cutting.hpp"

class NestView final : public QGraphicsView {
public:
    struct CuttingRouteOperation {
        std::size_t operation{};
        std::size_t sheetIndex{};
        std::string instanceId;
        bool inner{};
        std::size_t contourIndex{};
        double cutLengthMm{};
        double rapidLengthMm{};
        double estimatedSeconds{};
    };
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
    void setCuttingAnimationOperation(int operation);
    void setCuttingAnimationOperationProgress(int operation, double progress);
    const std::vector<CuttingRouteOperation>& cuttingRouteOperations() const {
        return cuttingRouteOperations_;
    }

protected:
    void wheelEvent(QWheelEvent* event) override;

private:
    bool cuttingRouteVisible_{false};
    double cuttingAnimationProgress_{1.0};
    int cuttingAnimationOperation_{-1};
    double cuttingAnimationOperationProgress_{0.0};
    std::vector<CuttingRouteOperation> cuttingRouteOperations_;

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
