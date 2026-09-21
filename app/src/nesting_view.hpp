#pragma once
#include "sheetnest/nesting.hpp"
#include <QGraphicsView>
#include <vector>

class NestingView final : public QGraphicsView {
public:
  explicit NestingView(QWidget* parent=nullptr);
  void setResult(const sheetnest::Result& result,const std::vector<sheetnest::Instance>& instances);
  void clearLayout();
  void fitLayout();

protected:
  void wheelEvent(QWheelEvent* event) override;
  void drawBackground(QPainter* painter,const QRectF& rect) override;

private:
  QGraphicsScene scene_;
  double pixelsPerMm_{0.35};
};
