#pragma once
#include <vector>
namespace sheetnest{struct Point{double x{},y{};};using Polygon=std::vector<Point>;struct Bounds{double minX{},minY{},maxX{},maxY{};double width()const{return maxX-minX;}double height()const{return maxY-minY;}};Bounds bounds(const Polygon&);double polygonArea(const Polygon&);bool pointInPolygon(const Point&,const Polygon&);bool polygonsIntersect(const Polygon&,const Polygon&);Polygon rotate(const Polygon&,int);Polygon translate(const Polygon&,double,double);}
