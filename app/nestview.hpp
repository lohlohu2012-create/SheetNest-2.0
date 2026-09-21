#pragma once

#include <QGraphicsView>

#include "sheetnest/nesting.hpp"

class NestView final : public QGraphicsView {
public:
    explicit NestView(QWidget* parent = nullptr);

    void showResult(
        const sheetnest::Result& result,
        const std::vector<sheetnest::Instance>& instances,
        const sheetnest::Sheet& sheet
    );

    void clearResult();

protected:
    void wheelEvent(QWheelEvent* event) override;

private:
    QGraphicsPathItem* addPartItem(
        QGraphicsItem* parent,
        const sheetnest::Instance& instance,
        const sheetnest::Placement& placement,
        std::size_t colorIndex
    );
};
