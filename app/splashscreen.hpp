#pragma once

#include <QWidget>
#include <QTimer>
#include <QElapsedTimer>

class SplashScreen final : public QWidget {
public:
    explicit SplashScreen(QWidget* parent = nullptr);
    bool finished() const { return finished_; }

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void advance();

    QTimer timer_;
    QElapsedTimer elapsed_;
    double progress_{0.0};
    bool finished_{false};
};
