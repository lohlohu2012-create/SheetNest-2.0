#include "nesting_view.hpp"
#include <QGraphicsPathItem>
#include <QGraphicsRectItem>
#include <QGraphicsSimpleTextItem>
#include <QPainter>
#include <QWheelEvent>
#include <QPen>
#include <QBrush>
#include <QFont>
#include <algorithm>
#include <unordered_map>

namespace {
QPainterPath toPath(const sheetnest::Shape& shape) {
  QPainterPath path;
  path.setFillRule(Qt::OddEvenFill);
  if(shape.outer.size()>=3) {
    QPolygonF outer;
    for(const auto& p:shape.outer) outer<<QPointF(p.x,p.y);
    path.addPolygon(outer);
  }
  for(const auto& hole:shape.holes) {
    if(hole.size()<3) continue;
    QPolygonF h;
    for(const auto& p:hole) h<<QPointF(p.x,p.y);
    path.addPolygon(h);
  }
  return path;
}
}

NestingView::NestingView(QWidget* parent):QGraphicsView(parent),scene_(this) {
  setScene(&scene_);
  setRenderHint(QPainter::Antialiasing,true);
  setRenderHint(QPainter::SmoothPixmapTransform,true);
  setDragMode(QGraphicsView::ScrollHandDrag);
  setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  setResizeAnchor(QGraphicsView::AnchorViewCenter);
  setBackgroundBrush(QColor("#151a1f"));
}

void NestingView::clearLayout() {
  scene_.clear();
  scene_.setSceneRect(0,0,1,1);
}

void NestingView::setResult(const sheetnest::Result& result,const std::vector<sheetnest::Instance>& instances) {
  clearLayout();

  std::unordered_map<std::string,const sheetnest::Instance*> byId;
  for(const auto& i:instances) byId[i.id]=&i;

  constexpr double sheetGap=250.0;
  double sceneWidth=0;
  double sceneHeight=0;

  for(size_t si=0;si<result.sheets.size();++si) {
    const double ox=si*(result.sheetWidthMm+sheetGap);
    const double oy=0.0;
    const double sw=result.sheetWidthMm>0?result.sheetWidthMm:1500.0;
    const double sh=result.sheetHeightMm>0?result.sheetHeightMm:3000.0;

    auto* sheetItem=scene_.addRect(QRectF(ox,oy,sw,sh),
      QPen(QColor("#7c8994"),2),QBrush(QColor("#30373d")));
    sheetItem->setZValue(-20);

    auto* title=scene_.addSimpleText(
      QString("Лист %1   %2 × %3 мм").arg(int(si+1)).arg(int(sw)).arg(int(sh)),
      QFont("Segoe UI",11,QFont::Bold));
    title->setBrush(QColor("#d8e0e6"));
    title->setPos(ox,oy-55);

    for(const auto& placement:result.sheets[si]) {
      auto it=byId.find(placement.id);
      if(it==byId.end()) continue;

      sheetnest::Shape shape=sheetnest::normalized(
        sheetnest::rotate(it->second->part.shape,placement.rotation));
      shape=sheetnest::translate(shape,placement.x,placement.y);
      QPainterPath p=toPath(shape);

      auto* item=scene_.addPath(p,QPen(QColor("#56d3c2"),1.0),
                                  QBrush(QColor("#19675f")));
      item->setZValue(0);

      auto* text=scene_.addSimpleText(
        QString::fromStdString(placement.id),QFont("Segoe UI",8,QFont::Bold));
      text->setBrush(QColor("#ffffff"));
      text->setPos(ox+placement.x,oy+placement.y);
      text->setZValue(5);
    }

    sceneWidth=std::max(sceneWidth,ox+sw);
    sceneHeight=std::max(sceneHeight,sh+90.0);
  }

  scene_.setSceneRect(-80,-80,sceneWidth+160,sceneHeight+160);
  fitLayout();
}

void NestingView::fitLayout() {
  if(scene_.items().isEmpty()) return;
  fitInView(scene_.sceneRect(),Qt::KeepAspectRatio);
}

void NestingView::wheelEvent(QWheelEvent* event) {
  const double factor=event->angleDelta().y()>0?1.15:1.0/1.15;
  scale(factor,factor);
  event->accept();
}

void NestingView::drawBackground(QPainter* painter,const QRectF& rect) {
  painter->fillRect(rect,QColor("#151a1f"));
  const double grid=50.0;
  const int left=int(std::floor(rect.left()/grid));
  const int right=int(std::ceil(rect.right()/grid));
  const int top=int(std::floor(rect.top()/grid));
  const int bottom=int(std::ceil(rect.bottom()/grid));
  QPen p(QColor("#20272d"),0.0);
  painter->setPen(p);
  for(int x=left;x<=right;++x)painter->drawLine(QPointF(x*grid,rect.top()),QPointF(x*grid,rect.bottom()));
  for(int y=top;y<=bottom;++y)painter->drawLine(QPointF(rect.left(),y*grid),QPointF(rect.right(),y*grid));
}
