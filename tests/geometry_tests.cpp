#include "sheetnest/geometry.hpp"
#include <cassert>
#include <cmath>
int main(){using namespace sheetnest;Polygon p{{0,0},{10,0},{10,5},{0,5}};assert(std::abs(polygonArea(p)-50)<1e-9);assert(pointInPolygon({5,2},p));}
