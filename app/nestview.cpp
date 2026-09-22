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
    cuttingRoute_.operations.clear();
    cuttingRoute_.moves.clear();
}

void NestView::setCuttingAnimationProgress(double progress) {
    cuttingAnimationProgress_ = std::clamp(progress, 0.0, 1.0);
    cuttingAnimationOperation_ = -1;
}

void NestView::setCuttingAnimationOperation(int operation) {
    cuttingAnimationOperation_ = std::max(-1, operation);
    cuttingAnimationOperationProgress_ = 0.0;
}

void NestView::setCuttingAnimationOperationProgress(
    int operation,
    double progress
) {
    cuttingAnimationOperation_ = std::max(-1, operation);
    cuttingAnimationOperationProgress_ = std::clamp(progress, 0.0, 1.0);
}

void NestView::setCuttingRoute(
    const sheetnest::CuttingPath& route,
    double sheetHeight
) {
    cuttingRoute_ = route;
    sheetHeight_ = std::max(0.0, sheetHeight);
    cuttingAnimationOperation_ = -1;
    cuttingAnimationOperationProgress_ = 0.0;
}

static QPainterPath routePath(
    const sheetnest::Polygon& polygon,
    double yOffset,
    double progress
) {
    QPainterPath path;
    if (polygon.empty()) return path;

    const double p = std::clamp(progress, 0.0, 1.0);
    if (p <= 0.0) return path;

    std::vector<double> lengths(polygon.size(), 0.0);
    double total = 0.0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const auto& a = polygon[i];
        const auto& b = polygon[(i + 1) % polygon.size()];
        total += std::hypot(a.x - b.x, a.y - b.y);
        lengths[i] = total;
    }

    if (total <= 1e-9) return path;
    const double target = total * p;

    path.moveTo(polygon.front().x, polygon.front().y + yOffset);
    double accumulated = 0.0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const auto& from = polygon[i];
        const auto& to = polygon[(i + 1) % polygon.size()];
        const double seg = std::hypot(to.x - from.x, to.y - from.y);
        if (accumulated + seg <= target + 1e-9) {
            path.lineTo(to.x, to.y + yOffset);
            accumulated += seg;
            continue;
        }

        const double remaining = target - accumulated;
        if (seg > 1e-9 && remaining > 0.0) {
            const double t = std::clamp(remaining / seg, 0.0, 1.0);
            path.lineTo(
                from.x + (to.x - from.x) * t,
                from.y + (to.y - from.y) * t + yOffset
            );
        }
        break;
    }
    return path;
}

void NestView::addCuttingRoute() {
    if (!cuttingRouteVisible_ || cuttingRoute_.operations.empty()) return;

    const QColor rapidColor("#f59e0b");
    const QColor innerColor("#22d3ee");
    const QColor outerColor("#22c55e");
    const QColor pierceColor("#f43f5e");
    const QColor headColor("#ffffff");

    const std::size_t opCount = cuttingRoute_.operations.size();
    const double global = std::clamp(cuttingAnimationProgress_, 0.0, 1.0);
    int selected = cuttingAnimationOperation_;

    std::size_t visibleCount = opCount;
    double currentProgress = 1.0;
    if (selected >= 0 &&
        static_cast<std::size_t>(selected) < opCount) {
        visibleCount = static_cast<std::size_t>(selected) + 1;
        currentProgress = cuttingAnimationOperationProgress_;
    } else if (global < 1.0) {
        const double scaled = global * static_cast<double>(opCount);
        visibleCount = std::min(
            opCount,
            static_cast<std::size_t>(std::floor(scaled)) + 1
        );
        currentProgress = scaled -
            std::floor(scaled);
        if (visibleCount == 0) visibleCount = 1;
    }

    Point head{};
    bool haveHead = false;
    for (std::size_t i = 0; i < visibleCount; ++i) {
        const auto& op = cuttingRoute_.operations[i];
        const double yOffset =
            static_cast<double>(op.sheetIndex) *
            (sheetHeight_ + 120.0);

        const double progress =
            (i + 1 < visibleCount) ? 1.0 : currentProgress;

        if (op.rapidLengthMm > 1e-9) {
            QPainterPath rapid;
            rapid.moveTo(
                op.rapidFrom.x,
                op.rapidFrom.y + yOffset
            );
            rapid.lineTo(
                op.start.x,
                op.start.y + yOffset
            );
            auto* item = scene()->addPath(
                rapid,
                QPen(rapidColor, 1.0, Qt::DashLine)
            );
            item->setZValue(900.0);
        }

        const QPainterPath contour =
            routePath(op.contour, yOffset, progress);
        auto* item = scene()->addPath(
            contour,
            QPen(
                op.inner ? innerColor : outerColor,
                selected == static_cast<int>(i) ? 3.0 : 1.5
            )
        );
        item->setZValue(901.0);

        if (progress > 0.0) {
            auto* pierce = scene()->addEllipse(
                op.start.x - 2.5,
                op.start.y + yOffset - 2.5,
                5.0,
                5.0,
                QPen(pierceColor),
                QBrush(Qt::NoBrush)
            );
            pierce->setZValue(902.0);
        }

        if (i + 1 == visibleCount) {
            const double p = std::clamp(progress, 0.0, 1.0);
            const auto& contour = op.contour;
            if (!contour.empty()) {
                const QPainterPath partial =
                    routePath(contour, yOffset, p);
                if (!partial.isEmpty()) {
                    const QPointF pos = partial.currentPosition();
                    auto* headItem = scene()->addEllipse(
                        pos.x() - 4.0,
                        pos.y() - 4.0,
                        8.0,
                        8.0,
                        QPen(headColor, 1.5),
                        QBrush(headColor)
                    );
                    headItem->setZValue(905.0);
                    head = {pos.x(), pos.y() - yOffset};
                    haveHead = true;
                }
            }
        }
    }

    if (!haveHead && !cuttingRoute_.operations.empty()) {
        const auto& op = cuttingRoute_.operations.back();
        const double yOffset =
            static_cast<double>(op.sheetIndex) *
            (sheetHeight_ + 120.0);
        auto* headItem = scene()->addEllipse(
            op.end.x - 4.0,
            op.end.y + yOffset - 4.0,
            8.0,
            8.0,
            QPen(headColor, 1.5),
            QBrush(headColor)
        );
        headItem->setZValue(905.0);
    }
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
        addCuttingRoute();
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
