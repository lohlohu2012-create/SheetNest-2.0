#include "nestview.hpp"

#include <QGraphicsPathItem>
#include <QGraphicsRectItem>
#include <QGraphicsSimpleTextItem>
#include <QPainterPath>
#include <QPen>
#include <QBrush>
#include <QWheelEvent>
#include <QPolygonF>

#include <cmath>
#include <unordered_map>

using namespace sheetnest;

namespace {

QPolygonF toPolygon(const Polygon& polygon) {
    QPolygonF result;
    result.reserve(static_cast<int>(polygon.size()));
    for (const auto& p : polygon) {
        result << QPointF(p.x, p.y);
    }
    return result;
}

QPainterPath makePartPath(
    const Instance& instance,
    const Placement& placement
) {
    QPainterPath path;
    path.setFillRule(Qt::OddEvenFill);

    const auto outer = translate(
        rotate(instance.part.outer, placement.rotation),
        placement.x,
        placement.y
    );
    path.addPolygon(toPolygon(outer));

    for (const auto& hole : instance.part.holes) {
        const auto transformedHole = translate(
            rotate(hole, placement.rotation),
            placement.x,
            placement.y
        );
        path.addPolygon(toPolygon(transformedHole));
    }

    return path;
}

QColor partColor(std::size_t index) {
    static const QColor colors[] = {
        QColor("#38bdf8"),
        QColor("#a78bfa"),
        QColor("#34d399"),
        QColor("#fbbf24"),
        QColor("#fb7185"),
        QColor("#60a5fa"),
        QColor("#c084fc"),
        QColor("#2dd4bf")
    };
    return colors[index % (sizeof(colors) / sizeof(colors[0]))];
}

} // namespace

NestView::NestView(QWidget* parent)
    : QGraphicsView(parent)
{
    auto* scene = new QGraphicsScene(this);
    scene->setBackgroundBrush(QColor("#10151b"));
    setScene(scene);

    setRenderHint(QPainter::Antialiasing, true);
    setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setResizeAnchor(QGraphicsView::AnchorViewCenter);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setFrameShape(QFrame::NoFrame);
}

void NestView::clearResult() {
    scene()->clear();
    scene()->setSceneRect(QRectF());
}

void NestView::showResult(
    const Result& result,
    const std::vector<Instance>& instances,
    const Sheet& sheet
) {
    clearResult();

    std::unordered_map<std::string, const Instance*> byId;
    byId.reserve(instances.size());
    for (const auto& instance : instances) {
        byId.emplace(instance.id, &instance);
    }

    constexpr double sheetGap = 120.0;
    double cursorY = 0.0;

    for (std::size_t sheetIndex = 0;
         sheetIndex < result.sheets.size();
         ++sheetIndex) {

        auto* sheetItem = new QGraphicsRectItem(
            0.0,
            0.0,
            sheet.width,
            sheet.height
        );
        sheetItem->setBrush(QBrush(QColor("#27313b")));
        sheetItem->setPen(QPen(QColor("#64748b"), 1.5));
        sheetItem->setFlag(
            QGraphicsItem::ItemIsMovable,
            true
        );
        sheetItem->setFlag(
            QGraphicsItem::ItemIsSelectable,
            true
        );
        sheetItem->setToolTip(
            QString("Лист %1 — %.0f × %.0f мм")
                .arg(static_cast<int>(sheetIndex + 1))
                .arg(sheet.width)
                .arg(sheet.height)
        );
        sheetItem->setPos(0.0, cursorY);
        scene()->addItem(sheetItem);

        auto* label = new QGraphicsSimpleTextItem(
            QString("Лист %1  •  %2 × %3 мм")
                .arg(static_cast<int>(sheetIndex + 1))
                .arg(sheet.width, 0, 'f', 0)
                .arg(sheet.height, 0, 'f', 0),
            sheetItem
        );
        label->setBrush(QBrush(QColor("#cbd5e1")));
        label->setPos(12.0, 8.0);

        const auto& placements = result.sheets[sheetIndex];

        for (std::size_t i = 0; i < placements.size(); ++i) {
            const auto& placement = placements[i];
            const auto it = byId.find(placement.id);
            if (it == byId.end()) continue;

            auto* partItem = addPartItem(
                sheetItem,
                *it->second,
                placement,
                i
            );
            partItem->setToolTip(
                QString::fromStdString(placement.id)
            );
        }

        cursorY += sheet.height + sheetGap;
    }

    if (!scene()->items().isEmpty()) {
        const QRectF all = scene()->itemsBoundingRect().adjusted(
            -80.0,
            -80.0,
            80.0,
            80.0
        );
        scene()->setSceneRect(all);
        fitInView(all, Qt::KeepAspectRatio);
    }
}

QGraphicsPathItem* NestView::addPartItem(
    QGraphicsItem* parent,
    const Instance& instance,
    const Placement& placement,
    std::size_t colorIndex
) {
    auto* item = new QGraphicsPathItem(
        makePartPath(instance, placement),
        parent
    );

    const QColor color = partColor(colorIndex);
    item->setBrush(QBrush(color));
    item->setPen(QPen(color.lighter(135), 0.7));

    return item;
}

void NestView::wheelEvent(QWheelEvent* event) {
    const double steps = static_cast<double>(
        event->angleDelta().y()
    ) / 120.0;

    if (std::abs(steps) < 1e-9) {
        event->accept();
        return;
    }

    const double factor = std::pow(1.15, steps);
    scale(factor, factor);
    event->accept();
}
