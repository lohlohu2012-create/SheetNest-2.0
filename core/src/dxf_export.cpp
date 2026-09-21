#include "sheetnest/dxf_export.hpp"
#include "sheetnest/geometry.hpp"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unordered_map>

namespace sheetnest {
namespace {
static void pair(std::ostringstream& out,int code,const std::string& value) {
  out<<code<<"\n"<<value<<"\n";
}
static void number(std::ostringstream& out,int code,double value) {
  out<<code<<"\n"<<std::setprecision(15)<<value<<"\n";
}
static void writePolyline(std::ostringstream& out,const Polygon& p,const std::string& layer) {
  if(p.size()<3)return;
  pair(out,0,"LWPOLYLINE");
  pair(out,8,layer);
  pair(out,90,std::to_string(p.size()));
  pair(out,70,"1");
  for(const auto& q:p){number(out,10,q.x);number(out,20,q.y);}
}
}
DxfExportResult exportNestDxf(const std::vector<Instance>& parts,const Result& result) {
  DxfExportResult out;
  std::unordered_map<std::string,const Instance*> byId;
  for(const auto& p:parts)byId[p.id]=&p;

  std::ostringstream ss;
  ss<<"0\nSECTION\n2\nHEADER\n";
  pair(ss,9,"$ACADVER"); pair(ss,1,"AC1015");
  ss<<"0\nENDSEC\n";
  ss<<"0\nSECTION\n2\nENTITIES\n";

  std::size_t sheetIndex=0;
  const double pitchX=result.sheetWidthMm>0?result.sheetWidthMm+100.0:10000.0;
  const double pitchY=result.sheetHeightMm>0?result.sheetHeightMm+100.0:0.0;
  for(const auto& sheet:result.sheets) {
    const double sheetOffsetX=static_cast<double>(sheetIndex)*pitchX;
    const double sheetOffsetY=0.0;
    ++sheetIndex;
    for(const auto& placement:sheet) {
      auto it=byId.find(placement.id);
      if(it==byId.end()) {
        out.diagnostics.push_back({placement.id,"Не найдена исходная Instance для Placement."});
        continue;
      }
      const Shape shape=normalized(rotate(it->second->part.shape,placement.rotation));
      Shape placed=translate(shape,placement.x+sheetOffsetX,placement.y+sheetOffsetY);
      const std::string layer="SHEET_"+std::to_string(sheetIndex);
      writePolyline(ss,placed.outer,layer);
      for(const auto& hole:placed.holes)writePolyline(ss,hole,layer+"_HOLES");
      ++out.exportedPlacements;
    }
  }

  ss<<"0\nENDSEC\n0\nEOF\n";
  out.text=ss.str();
  return out;
}
bool exportNestDxfFile(const std::string& path,const std::vector<Instance>& parts,const Result& result) {
  const auto exported=exportNestDxf(parts,result);
  std::ofstream file(path,std::ios::binary);
  if(!file)return false;
  file<<exported.text;
  return file.good();
}
}
