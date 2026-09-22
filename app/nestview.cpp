#include "nestview.hpp"

#include <QGraphicsPathItem>
#include <QGraphicsRectItem>
#include <QGraphicsSimpleTextItem>
#include <QPainterPath>
#include <QPen>
#include <QBrush>
#include <QWheelEvent>
#include <QPolygonF>\n#include <QGraphicsLineItem>\n#include <QGraphicsEllipseItem>

#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <vector>\n#include <algorithm>\n#include <limits>

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
    const QString& roundLabel,
    std::size_t conflictCount,
    std::size_t extractedCount,
    std::size_t movedCount,
    std::size_t stationaryCount
) {
    if (!scene ||
        (conflictCount == 0 &&
         extractedCount == 0 &&
         movedCount == 0 &&
         stationaryCount == 0)) {
        return;
    }

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
        QString("ADAPTIVE DESTROY-AND-REPAIR • %1")
            .arg(roundLabel)
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
        QString("Группа: %1 • извлечено: %2 • перемещено: %3 • "
                "неподвижно: %4")
            .arg(static_cast<qulonglong>(conflictCount))
            .arg(static_cast<qulonglong>(extractedCount))
            .arg(static_cast<qulonglong>(movedCount))
            .arg(static_cast<qulonglong>(stationaryCount))
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

void NestView::addCuttingRoute(
    const Result& result,
    const std::vector<Instance>& instances,
    const Sheet& sheet
) {
    std::unordered_map<std::string, const Instance*> byId;
    for (const auto& instance : instances) byId.emplace(instance.id, &instance);
    const QColor rapidColor("#f59e0b");
    const QColor holeColor("#22d3ee");
    const QColor outerColor("#22c55e");
    const QColor pierceColor("#f43f5e");
    const double pi = std::acos(-1.0);

    // One route overlay per sheet. The order is the same nearest-contour
    // strategy used by the CAM planner; every operation is still drawn from
    // the exact transformed nesting geometry.
    for (std::size_t sheetIndex = 0; sheetIndex < result.sheets.size(); ++sheetIndex) {
        const auto& placements = result.sheets[sheetIndex];
        if (placements.empty()) continue;
        struct RouteContour { Polygon polygon; bool inner{}; };
        std::vector<RouteContour> contours;
        for (const auto& placement : placements) {
            const auto it = byId.find(placement.id);
            if (it == byId.end()) continue;
            const auto& inst = *it->second;
            contours.push_back({translate(rotate(inst.part.outer, placement.rotation), placement.x, placement.y), false});
            for (const auto& hole : inst.part.holes) {
                contours.push_back({translate(rotate(hole, placement.rotation), placement.x, placement.y), true});
            }
        }
        if (contours.empty()) continue;

        // Locate the sheet rectangle by its stable vertical position.
        QGraphicsItem* sheetItem = nullptr;
        const double expectedY = sheetIndex * (sheet.height + 120.0);
        for (auto* item : scene()->items()) {
            auto* rect = dynamic_cast<QGraphicsRectItem*>(item);
            if (!rect || rect->parentItem() != nullptr) continue;
            if (std::abs(rect->rect().width() - sheet.width) < 1e-6 &&
                std::abs(rect->rect().height() - sheet.height) < 1e-6 &&
                std::abs(rect->pos().y() - expectedY) < 1e-6) {
                sheetItem = rect;
                break;
            }
        }
        if (!sheetItem) continue;

        Point head{};
        std::vector<bool> used(contours.size(), false);
        for (std::size_t operation = 0; operation < contours.size(); ++operation) {
            std::size_t best = contours.size();
            double bestDistance = std::numeric_limits<double>::infinity();
            bool hasUnusedInner = false;
            for (std::size_t i = 0; i < contours.size(); ++i) {
                if (!used[i] && contours[i].inner && !contours[i].polygon.empty()) {
                    hasUnusedInner = true;
                    break;
                }
            }
            for (std::size_t i = 0; i < contours.size(); ++i) {
                if (used[i] || contours[i].polygon.empty()) continue;
                if (hasUnusedInner && !contours[i].inner) continue;
                const auto& p = contours[i].polygon.front();
                const double distance = std::hypot(head.x - p.x, head.y - p.y);
                if (distance < bestDistance) { bestDistance = distance; best = i; }
            }
            if (best == contours.size()) break;
            used[best] = true;
            const auto& contour = contours[best];
            const Point start = contour.polygon.front();

            if (bestDistance > 1e-9) {
                auto* rapid = new QGraphicsLineItem(
                    QLineF(QPointF(head.x, head.y), QPointF(start.x, start.y)), sheetItem);
                QPen pen(rapidColor, 1.4, Qt::DashLine);
                pen.setCosmetic(true);
                rapid->setPen(pen);
                rapid->setZValue(40.0);
                rapid->setToolTip(QString("RAPID-переход • %1 мм").arg(bestDistance, 0, 'f', 1));
                const QPointF mid((head.x + start.x) * .5, (head.y + start.y) * .5);
                auto* arrow = new QGraphicsSimpleTextItem("➜", sheetItem);
                arrow->setBrush(QBrush(rapidColor));
                arrow->setPos(mid);
                arrow->setRotation(std::atan2(start.y - head.y, start.x - head.x) * 180.0 / pi);
                arrow->setZValue(42.0);
            }

            auto* pierce = new QGraphicsEllipseItem(start.x - 2.4, start.y - 2.4, 4.8, 4.8, sheetItem);
            pierce->setPen(QPen(pierceColor, 1.2));
            pierce->setBrush(QBrush(pierceColor));
            pierce->setZValue(45.0);
            pierce->setToolTip(QString("Пробивка №%1 • %2").arg(operation + 1).arg(contour.inner ? "отверстие" : "внешний контур"));

            const QColor color = contour.inner ? holeColor : outerColor;
            for (std::size_t i = 0; i < contour.polygon.size(); ++i) {
                const auto& a = contour.polygon[i];
                const auto& b = contour.polygon[(i + 1) % contour.polygon.size()];
                auto* cut = new QGraphicsLineItem(QLineF(QPointF(a.x, a.y), QPointF(b.x, b.y)), sheetItem);
                QPen pen(color, 2.0);
                pen.setCosmetic(true);
                cut->setPen(pen);
                cut->setZValue(41.0);
                cut->setToolTip(QString("CUT • %1 • операция %2")
                    .arg(contour.inner ? "внутренний контур" : "внешний контур")
                    .arg(operation + 1));
            }
            auto* number = new QGraphicsSimpleTextItem(QString::number(operation + 1), sheetItem);
            number->setBrush(QBrush(color));
            number->setPos(start.x + 5.0, start.y + 5.0);
            number->setZValue(46.0);
            head = start;
        }
    }

    auto* legend = scene()->addSimpleText(
        "LASER ROUTE  |  ● пробивка  |  голубой внутренний  |  зелёный внешний  |  - - rapid  |  ➜ направление");
    legend->setBrush(QBrush(QColor("#f8fafc")));
    legend->setZValue(1200.0);
    legend->setPos(18.0, -42.0);
}

void NestView::showResult(
    const Result& result,
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const sheetnest::ProductionValidationReport* repairVisualization,
    int repairRound,
    bool showConflict,
    bool showExtracted,
    bool showMoved,
    bool showStationary,
    int animationStage
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

    const std::vector<sheetnest::AdaptiveRepairChange>* selectedChanges =
        nullptr;
    if (repairVisualization &&
        repairVisualization->repaired) {
        if (repairRound > 0) {
            for (const auto& history :
                 repairVisualization->adaptiveHistory) {
                if (static_cast<int>(history.roundIndex) ==
                    repairRound) {
                    selectedChanges = &history.changes;
                    break;
                }
            }
        } else if (!repairVisualization->adaptiveChanges.empty()) {
            selectedChanges = &repairVisualization->adaptiveChanges;
        }
    }

    const bool animated =
        repairRound > 0 &&
        animationStage >= 0 &&
        animationStage <= 3;

    Result displayResult = result;

    if (repairVisualization &&
        repairRound > 0) {
        for (const auto& history :
             repairVisualization->adaptiveHistory) {
            if (static_cast<int>(history.roundIndex) ==
                repairRound) {
                if (animated &&
                    (animationStage == 0 ||
                     animationStage == 1)) {
                    displayResult.sheets = history.beforeSheets;
                } else {
                    displayResult.sheets = history.afterSheets;
                }
                break;
            }
        }
    }

    const bool hasAdaptiveRepair =
        selectedChanges && !selectedChanges->empty();

    if (hasAdaptiveRepair) {
        for (const auto& change : *selectedChanges) {
            repairChangesById[change.after.id] = change;
            repairGhostsBySheet[change.sheetIndex].push_back(change);
        }
    }

    constexpr double sheetGap = 120.0;
    double cursorY = 0.0;

    for (std::size_t sheetIndex = 0;
         sheetIndex < displayResult.sheets.size();
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
                    if (!change.extracted || !showExtracted) continue;
                    if (animated && animationStage == 0) continue;

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

        const auto& placements = displayResult.sheets[sheetIndex];

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

                const bool stageShowsMoved =
                    !animated ||
                    animationStage == 2 ||
                    animationStage == 3;
                const bool stageShowsStationary =
                    !animated ||
                    animationStage == 2 ||
                    animationStage == 3;
                const bool stageShowsConflict =
                    !animated ||
                    animationStage == 0 ||
                    animationStage == 1 ||
                    animationStage == 3;

                if (change.moved &&
                    showMoved &&
                    stageShowsMoved) {
                    addRepairOverlay(
                        sheetItem,
                        *it->second,
                        placement,
                        QColor("#22d3ee"),
                        Qt::SolidLine,
                        2.8,
                        true
                    )->setToolTip(
                        QString("Новая позиция: %1")
                            .arg(QString::fromStdString(
                                placement.id))
                    );
                }

                if (change.stationary &&
                    showStationary &&
                    stageShowsStationary) {
                    addRepairOverlay(
                        sheetItem,
                        *it->second,
                        placement,
                        QColor("#94a3b8"),
                        Qt::DashDotLine,
                        1.7,
                        true
                    )->setToolTip(
                        QString("Осталась неподвижной: %1")
                            .arg(QString::fromStdString(
                                placement.id))
                    );
                }

                if (change.conflictGroup &&
                    showConflict &&
                    stageShowsConflict) {
                    addRepairOverlay(
                        sheetItem,
                        *it->second,
                        placement,
                        QColor("#ff4d5e"),
                        Qt::SolidLine,
                        3.4,
                        true
                    )->setToolTip(
                        QString("Конфликтующая группа: %1")
                            .arg(QString::fromStdString(
                                placement.id))
                    );
                }

                if (animated &&
                    animationStage == 1 &&
                    change.extracted) {
                    partItem->setOpacity(0.18);
                } else if (animated &&
                           animationStage == 0 &&
                           change.conflictGroup) {
                    partItem->setOpacity(0.72);
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

    if (cuttingRouteVisible_) {
        addCuttingRoute(displayResult, instances, sheet);
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
            if (animated) {
                const QString titles[] = {
                    "1/4  КОНФЛИКТ",
                    "2/4  ИЗВЛЕЧЕНИЕ ДЕТАЛЕЙ",
                    "3/4  ЛОКАЛЬНАЯ ПЕРЕПАКОВКА",
                    "4/4  PRODUCTION VALIDATOR"
                };
                const QColor colors[] = {
                    QColor("#ff4d5e"),
                    QColor("#facc15"),
                    QColor("#22d3ee"),
                    QColor("#22c55e")
                };
                const QString detail =
                    animationStage == 0
                        ? QString("Раунд %1 • конфликтов: %2")
                            .arg(repairRound)
                            .arg(static_cast<qulonglong>(
                                repairVisualization->adaptiveConflictIds.size()))
                        : animationStage == 1
                            ? QString("Раунд %1 • извлечение: %2")
                                .arg(repairRound)
                                .arg(static_cast<qulonglong>(
                                    repairVisualization->adaptiveExtractedIds.size()))
                            : animationStage == 2
                                ? QString("Раунд %1 • перемещено: %2")
                                    .arg(repairRound)
                                    .arg(static_cast<qulonglong>(
                                        repairVisualization->adaptiveMovedIds.size()))
                                : QString("Раунд %1 • Validator: %2")
                                    .arg(repairRound)
                                    .arg(repairVisualization->valid
                                             ? "PASS"
                                             : "FAIL");

                const QRectF bannerRect(
                    scene()->sceneRect().right() - 380.0,
                    scene()->sceneRect().top() + 18.0,
                    340.0,
                    72.0
                );
                auto* banner = scene()->addRect(
                    bannerRect,
                    QPen(colors[animationStage], 1.8),
                    QBrush(QColor(12, 18, 24, 235))
                );
                banner->setZValue(1100.0);

                auto* title = scene()->addSimpleText(
                    titles[animationStage]
                );
                title->setBrush(QBrush(colors[animationStage]));
                title->setPos(
                    bannerRect.left() + 14.0,
                    bannerRect.top() + 10.0
                );
                title->setZValue(1101.0);

                auto* detailItem = scene()->addSimpleText(detail);
                detailItem->setBrush(QBrush(QColor("#cbd5e1")));
                detailItem->setPos(
                    bannerRect.left() + 14.0,
                    bannerRect.top() + 38.0
                );
                detailItem->setZValue(1101.0);
            }

            QString roundLabel = "Итог";
            std::size_t conflictCount = 0;
            std::size_t extractedCount = 0;
            std::size_t movedCount = 0;
            std::size_t stationaryCount = 0;

            if (repairRound > 0) {
                for (const auto& history :
                     repairVisualization->adaptiveHistory) {
                    if (static_cast<int>(history.roundIndex) !=
                        repairRound) {
                        continue;
                    }
                    roundLabel =
                        QString("Раунд %1")
                            .arg(repairRound);
                    conflictCount = history.conflictIds.size();
                    extractedCount = history.extractedIds.size();
                    movedCount = history.movedIds.size();
                    stationaryCount = history.stationaryIds.size();
                    break;
                }
            } else {
                roundLabel = "Итог";
                conflictCount =
                    repairVisualization->adaptiveConflictIds.size();
                extractedCount =
                    repairVisualization->adaptiveExtractedIds.size();
                movedCount =
                    repairVisualization->adaptiveMovedIds.size();
                stationaryCount =
                    repairVisualization->adaptiveStationaryIds.size();
            }

            addRepairLegend(
                scene(),
                scene()->sceneRect(),
                roundLabel,
                conflictCount,
                extractedCount,
                movedCount,
                stationaryCount
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
