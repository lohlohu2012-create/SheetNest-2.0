#include "nestview.hpp"

#include <QGraphicsPathItem>
#include <QGraphicsRectItem>
#include <QGraphicsSimpleTextItem>
#include <QPainterPath>
#include <QPen>
#include <QBrush>
#include <QWheelEvent>
#include <QPolygonF>
#include <QGraphicsLineItem>

#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <vector>

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

QColor repairColor(bool conflict, bool moved, bool stationary) {
    if (conflict) return QColor("#ff4d5e");
    if (moved) return QColor("#22d3ee");
    if (stationary) return QColor("#94a3b8");
    return QColor("#facc15");
}

QGraphicsPathItem* addRepairOverlay(
    QGraphicsItem* parent,
    const Instance& instance,
    const Placement& placement,
    const QColor& color,
    Qt::PenStyle style,
    qreal width,
    bool ghost
) {
    auto* overlay = new QGraphicsPathItem(
        makePartPath(instance, placement),
        parent
    );
    overlay->setBrush(
        ghost
            ? QBrush(QColor(0, 0, 0, 0))
            : QBrush(Qt::NoBrush)
    );
    QPen pen(color, width, style);
    pen.setCosmetic(true);
    overlay->setPen(pen);
    overlay->setOpacity(ghost ? 0.9 : 1.0);
    overlay->setZValue(10.0);
    return overlay;
}

void addRepairLegend(
    QGraphicsScene* scene,
    const QRectF& sceneRect,
    const sheetnest::ProductionValidationReport& report
) {
    if (!scene || report.adaptiveChanges.empty()) return;

    auto* panel = scene->addRect(
        sceneRect.left() + 18.0,
        sceneRect.top() + 18.0,
        270.0,
        142.0,
        QPen(QColor("#475569"), 1.0),
        QBrush(QColor(12, 18, 24, 235))
    );
    panel->setZValue(1000.0);
    panel->setToolTip(
        "Adaptive Destroy-and-Repair: локальная перепаковка"
    );

    auto* title = scene->addSimpleText(
        "ADAPTIVE DESTROY-AND-REPAIR"
    );
    title->setBrush(QBrush(QColor("#f8fafc")));
    title->setPos(
        sceneRect.left() + 32.0,
        sceneRect.top() + 28.0
    );
    title->setZValue(1001.0);

    const struct Entry {
        const char* text;
        QColor color;
        Qt::PenStyle style;
    } entries[] = {
        {"Конфликтующая группа", QColor("#ff4d5e"), Qt::SolidLine},
        {"Извлечённая старая позиция", QColor("#facc15"), Qt::DashLine},
        {"Новая позиция", QColor("#22d3ee"), Qt::SolidLine},
        {"Неподвижная деталь", QColor("#94a3b8"), Qt::DashDotLine}
    };

    for (int i = 0; i < 4; ++i) {
        const double y = sceneRect.top() + 55.0 + i * 22.0;

        auto* swatch = scene->addRect(
            sceneRect.left() + 32.0,
            y,
            20.0,
            11.0,
            QPen(entries[i].color, 2.0),
            QBrush(Qt::NoBrush)
        );
        swatch->setZValue(1001.0);

        auto* label = scene->addSimpleText(entries[i].text);
        label->setBrush(QBrush(QColor("#cbd5e1")));
        label->setPos(
            sceneRect.left() + 62.0,
            y - 5.0
        );
        label->setZValue(1001.0);
    }

    auto* stats = scene->addSimpleText(
        QString("Группа: %1 • извлечено: %2 • перемещено: %3")
            .arg(static_cast<qulonglong>(
                report.adaptiveConflictIds.size()))
            .arg(static_cast<qulonglong>(
                report.adaptiveExtractedIds.size()))
            .arg(static_cast<qulonglong>(
                report.adaptiveMovedIds.size()))
    );
    stats->setBrush(QBrush(QColor("#94a3b8")));
    stats->setPos(
        sceneRect.left() + 32.0,
        sceneRect.top() + 136.0
    );
    stats->setZValue(1001.0);
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

    std::unordered_map<std::string, sheetnest::AdaptiveRepairChange>
        repairChangesById;
    std::unordered_map<std::size_t,
                       std::vector<sheetnest::AdaptiveRepairChange>>
        repairGhostsBySheet;

    const bool hasAdaptiveRepair =
        repairVisualization &&
        repairVisualization->repaired &&
        !repairVisualization->adaptiveChanges.empty();

    if (hasAdaptiveRepair) {
        for (const auto& change :
             repairVisualization->adaptiveChanges) {
            repairChangesById[change.after.id] = change;
            repairGhostsBySheet[change.sheetIndex].push_back(change);
        }
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

        if (hasAdaptiveRepair) {
            const auto ghostIt =
                repairGhostsBySheet.find(sheetIndex);

            if (ghostIt != repairGhostsBySheet.end()) {
                for (const auto& change : ghostIt->second) {
                    const auto instanceIt =
                        byId.find(change.before.id);

                    if (instanceIt == byId.end()) continue;
                    if (!change.extracted) continue;

                    addRepairOverlay(
                        sheetItem,
                        *instanceIt->second,
                        change.before,
                        QColor("#facc15"),
                        Qt::DashLine,
                        2.4,
                        true
                    )->setToolTip(
                        QString("Извлечённая деталь: %1")
                            .arg(QString::fromStdString(
                                change.before.id))
                    );
                }
            }
        }

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

            const auto changeIt =
                repairChangesById.find(placement.id);

            if (hasAdaptiveRepair &&
                changeIt != repairChangesById.end()) {
                const auto& change = changeIt->second;

                QColor baseColor = partColor(i);
                if (change.stationary) {
                    baseColor = QColor("#475569");
                    partItem->setOpacity(0.62);
                } else if (change.moved) {
                    baseColor = QColor("#10b981");
                }

                partItem->setBrush(
                    QBrush(baseColor)
                );

                QPen basePen(
                    change.stationary
                        ? QColor("#94a3b8")
                        : QColor("#34d399"),
                    change.stationary ? 1.2 : 2.0
                );
                basePen.setCosmetic(true);
                partItem->setPen(basePen);

                if (change.moved) {
                    addRepairOverlay(
                        sheetItem,
                        *it->second,
                        placement,
                        QColor("#22d3ee"),
                        Qt::SolidLine,
                        2.6,
                        true
                    )->setToolTip(
                        QString("Новая позиция: %1")
                            .arg(QString::fromStdString(
                                placement.id))
                    );
                } else if (change.stationary) {
                    addRepairOverlay(
                        sheetItem,
                        *it->second,
                        placement,
                        QColor("#94a3b8"),
                        Qt::DashDotLine,
                        1.5,
                        true
                    )->setToolTip(
                        QString("Осталась неподвижной: %1")
                            .arg(QString::fromStdString(
                                placement.id))
                    );
                }

                if (change.conflictGroup) {
                    addRepairOverlay(
                        sheetItem,
                        *it->second,
                        placement,
                        QColor("#ff4d5e"),
                        Qt::SolidLine,
                        3.2,
                        true
                    )->setToolTip(
                        QString("Конфликтующая группа: %1")
                            .arg(QString::fromStdString(
                                placement.id))
                    );
                }

                partItem->setToolTip(
                    QString("%1\nAdaptive Repair: %2")
                        .arg(QString::fromStdString(placement.id))
                        .arg(
                            change.moved
                                ? "перемещена"
                                : "неподвижна"
                        )
                );
            } else {
                partItem->setToolTip(
                    QString::fromStdString(placement.id)
                );
            }
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

        if (hasAdaptiveRepair) {
            addRepairLegend(
                scene(),
                scene()->sceneRect(),
                *repairVisualization
            );
        }

        fitInView(
            scene()->sceneRect(),
            Qt::KeepAspectRatio
        );
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
