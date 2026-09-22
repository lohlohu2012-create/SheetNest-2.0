#include "splashscreen.hpp"

#include <QApplication>
#include <QPainter>
#include <QPainterPath>
#include <QScreen>

#include <algorithm>
#include <cmath>

SplashScreen::SplashScreen(QWidget* parent)
    : QWidget(parent) {
    setWindowFlags(
        Qt::FramelessWindowHint |
        Qt::WindowStaysOnTopHint |
        Qt::SplashScreen
    );
    setAttribute(Qt::WA_TranslucentBackground, false);
    resize(1010, 588);

    if (const auto* screen = QApplication::primaryScreen()) {
        const QRect area = screen->availableGeometry();
        move(
            area.center().x() - width() / 2,
            area.center().y() - height() / 2
        );
    }

    elapsed_.start();
    connect(&timer_, &QTimer::timeout, this, &SplashScreen::advance);
    timer_.start(33);
}

void SplashScreen::advance() {
    constexpr double durationMs = 4200.0;
    progress_ = std::clamp(
        static_cast<double>(elapsed_.elapsed()) / durationMs,
        0.0,
        1.0
    );
    if (progress_ >= 1.0) {
        finished_ = true;
        timer_.stop();
    }
    update();
}

void SplashScreen::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    const QRectF r = rect();
    p.fillRect(r, QColor("#020608"));

    // Cyan industrial light, matching the supplied CAM intro.
    const QPointF glowCenter(
        -40.0 + progress_ * 130.0,
        height() * 0.62
    );
    for (int i = 16; i >= 1; --i) {
        const qreal radius = 180.0 + i * 42.0;
        const int alpha = std::max(0, 34 - i * 2);
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(0, 180, 205, alpha));
        p.drawEllipse(glowCenter, radius, radius * 0.72);
    }

    // Perspective plate / nesting field.
    QPolygonF plate;
    const qreal sx = width() * 0.10;
    const qreal sy = height() * 0.22;
    plate << QPointF(sx, sy + 65)
          << QPointF(width() * 0.72, sy + 25)
          << QPointF(width() * 0.91, sy + 150)
          << QPointF(width() * 0.83, height() * 0.80)
          << QPointF(width() * 0.20, height() * 0.78)
          << QPointF(sx - 35, height() * 0.50);

    p.setPen(QPen(QColor(18, 73, 82, 210), 1.0));
    p.setBrush(QColor(5, 16, 19, 210));
    p.drawPolygon(plate);

    auto drawPart = [&](const QPolygonF& poly, double phase) {
        QPolygonF q = poly;
        const qreal dx = std::sin((progress_ + phase) * 3.0) * 2.0;
        const qreal dy = std::cos((progress_ + phase) * 2.0) * 1.5;
        for (auto& point : q) point += QPointF(dx, dy);
        p.setPen(QPen(QColor(38, 102, 112, 210), 1.0));
        p.setBrush(QColor(4, 11, 14, 230));
        p.drawPolygon(q);
    };

    drawPart(
        {QPointF(180, 260), QPointF(360, 240), QPointF(340, 340),
         QPointF(190, 360)},
        0.0
    );
    drawPart(
        {QPointF(410, 270), QPointF(600, 245), QPointF(690, 360),
         QPointF(470, 380)},
        0.8
    );
    drawPart(
        {QPointF(290, 385), QPointF(470, 375), QPointF(520, 500),
         QPointF(260, 490)},
        1.6
    );

    // Orange cutting trajectory.
    QPainterPath route;
    route.moveTo(165, 300);
    route.lineTo(350, 270);
    route.lineTo(430, 320);
    route.lineTo(600, 285);
    route.lineTo(730, 400);
    route.lineTo(520, 465);
    route.lineTo(310, 450);

    const qreal totalLength = 1300.0;
    const qreal target = totalLength * progress_;
    QPainterPath partial;
    partial.moveTo(route.pointAtPercent(0));
    qreal last = 0.0;
    for (int i = 1; i <= 100; ++i) {
        const qreal t = static_cast<qreal>(i) / 100.0;
        const QPointF pt = route.pointAtPercent(t);
        const QPointF prev = route.pointAtPercent(last);
        const qreal segment = QLineF(prev, pt).length();
        if (target >= segment) {
            partial.lineTo(pt);
            last = t;
        } else {
            const qreal k = segment > 0.0 ? target / segment : 0.0;
            partial.lineTo(
                prev + (pt - prev) * k
            );
            break;
        }
    }

    p.setPen(QPen(QColor("#f59e0b"), 2.0));
    p.setBrush(Qt::NoBrush);
    p.drawPath(partial);

    // Animated laser head.
    if (!partial.isEmpty()) {
        const QPointF head = partial.currentPosition();
        p.setPen(QPen(QColor("#fff7ed"), 1.5));
        p.setBrush(QColor("#f59e0b"));
        p.drawEllipse(head, 5.0, 5.0);
    }

    p.setPen(QColor("#67e8f9"));
    QFont title = p.font();
    title.setBold(true);
    title.setPointSize(34);
    p.setFont(title);
    p.drawText(
        QRectF(0, 45, width(), 55),
        Qt::AlignCenter,
        "SheetNest 2.0"
    );

    QFont subtitle = p.font();
    subtitle.setBold(false);
    subtitle.setPointSize(11);
    p.setFont(subtitle);
    p.setPen(QColor("#647c82"));
    p.drawText(
        QRectF(0, 96, width(), 28),
        Qt::AlignCenter,
        "GENERATIVE INDUSTRIAL CAM & NESTING ENGINE"
    );

    p.setPen(QColor("#3d555b"));
    p.drawText(
        QRectF(0, height() - 55, width(), 24),
        Qt::AlignCenter,
        "DXF • TRUE-SHAPE NESTING • NFP • LASER CAM"
    );

    p.setPen(QPen(QColor(30, 55, 61), 2));
    p.drawLine(
        QPointF(width() * 0.25, height() - 28),
        QPointF(width() * 0.75, height() - 28)
    );
    p.setPen(QPen(QColor("#22d3ee"), 2));
    p.drawLine(
        QPointF(width() * 0.25, height() - 28),
        QPointF(width() * (0.25 + 0.5 * progress_), height() - 28)
    );
}
