#include "sheetnest/dxf.hpp"
#include <cassert>
int main(){auto d=sheetnest::importDxf("0\nSECTION\n0\nCIRCLE\n10\n50\n20\n50\n40\n10\n0\nENDSEC\n");assert(d.contours.size()==1);assert(d.contours[0].outer.size()>20);}
