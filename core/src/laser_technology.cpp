#include "sheetnest/laser_technology.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <utility>

namespace sheetnest {
namespace {
static std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::string current;
  bool quoted=false;
  for(size_t i=0;i<line.size();++i) {
    const char c=line[i];
    if(c=='"') {
      if(quoted && i+1<line.size() && line[i+1]=='"') {
        current.push_back('"');
        ++i;
      } else {
        quoted=!quoted;
      }
    } else if(c==','&&!quoted) {
      out.push_back(current);
      current.clear();
    } else {
      current.push_back(c);
    }
  }
  out.push_back(current);
  return out;
}
static double number(const std::string& s) {
  try { return std::stod(s); } catch(...) { return 0.0; }
}
static double midpoint(double a,double b) { return (a+b)*0.5; }
static std::string csvQuote(const std::string& value) {
  std::string escaped;
  escaped.reserve(value.size()+8);
  for(char c:value) {
    if(c=='"') escaped += """";
    else escaped += c;
  }
  return """ + escaped + """;
}
}

bool LaserTechnologyDatabase::loadCsv(const std::string& path) {
  std::ifstream file(path);
  if(!file) return false;

  std::string line;
  bool firstNonEmpty=true;
  std::vector<LaserTechnologyPoint> loaded;

  while(std::getline(file,line)) {
    if(!line.empty() && line.back()=='\r') line.pop_back();
    if(line.empty()) continue;

    if(firstNonEmpty) {
      firstNonEmpty=false;
      if(line.size()>=3 &&
         static_cast<unsigned char>(line[0])==0xEF &&
         static_cast<unsigned char>(line[1])==0xBB &&
         static_cast<unsigned char>(line[2])==0xBF) {
        line.erase(0,3);
      }
      if(line.rfind("laser_power_kw,",0)==0) continue;
    }

    const auto c=splitCsv(line);
    if(c.size()<10) continue;

    LaserTechnologyPoint p;
    p.laserPowerKw=number(c[0]);
    p.material=c[1];
    p.thicknessMm=number(c[2]);
    p.gas=c[3];
    p.speedMinMMin=number(c[4]);
    p.speedMaxMMin=number(c[5]);
    p.speedMMin=number(c[6]);
    p.pierceSeconds=number(c[7]);
    p.source=c[8];
    p.note=c[9];

    if(p.speedMMin<=0) p.speedMMin=midpoint(p.speedMinMMin,p.speedMaxMMin);
    if(p.speedMaxMMin<=0) p.speedMaxMMin=p.speedMMin;
    if(p.speedMinMMin<=0) p.speedMinMMin=p.speedMMin;
    if(p.pierceSeconds<=0) p.pierceSeconds=0.25;
    if(p.thicknessMm<=0||p.speedMMin<=0||p.speedMaxMMin<p.speedMinMMin) continue;

    loaded.push_back(std::move(p));
  }

  if(loaded.empty()) return false;
  points_=std::move(loaded);
  return true;
}

bool LaserTechnologyDatabase::saveCsv(const std::string& path) const {
  std::ofstream file(path,std::ios::trunc);
  if(!file) return false;

  file<<"laser_power_kw,material,thickness_mm,gas,speed_min_m_min,speed_max_m_min,speed_m_min,pierce_seconds,source,note\n";
  for(const auto& p:points_) {
    file<<p.laserPowerKw<<','<<csvQuote(p.material)<<','<<p.thicknessMm<<','<<csvQuote(p.gas)<<','
        <<p.speedMinMMin<<','<<p.speedMaxMMin<<','<<p.speedMMin<<','<<p.pierceSeconds<<','
        <<csvQuote(p.source)<<','<<csvQuote(p.note)<<"\n";
  }
  return file.good();
}

void LaserTechnologyDatabase::setPoints(std::vector<LaserTechnologyPoint> points) {
  points_=std::move(points);
}

std::optional<LaserTechnologyPoint> LaserTechnologyDatabase::lookup(
    const std::string& material,double thicknessMm,const std::string& gas,double laserPowerKw) const {
  if(points_.empty()||thicknessMm<=0) return std::nullopt;

  const LaserTechnologyPoint* lower=nullptr;
  const LaserTechnologyPoint* upper=nullptr;
  double bestDist=1e100;
  const LaserTechnologyPoint* nearest=nullptr;

  for(const auto& p:points_) {
    if(std::abs(p.laserPowerKw-laserPowerKw)>1e-6) continue;
    if(p.material!=material||p.gas!=gas) continue;

    const double d=std::abs(p.thicknessMm-thicknessMm);
    if(d<bestDist) { bestDist=d; nearest=&p; }

    if(p.thicknessMm<=thicknessMm && (!lower||p.thicknessMm>lower->thicknessMm))
      lower=&p;
    if(p.thicknessMm>=thicknessMm && (!upper||p.thicknessMm<upper->thicknessMm))
      upper=&p;
  }

  if(!nearest) return std::nullopt;

  if(lower&&upper&&lower!=upper) {
    const double t=(thicknessMm-lower->thicknessMm)/(upper->thicknessMm-lower->thicknessMm);
    LaserTechnologyPoint out=*lower;
    out.thicknessMm=thicknessMm;
    auto lerp=[&](double a,double b){return a+(b-a)*t;};
    out.speedMinMMin=lerp(lower->speedMinMMin,upper->speedMinMMin);
    out.speedMaxMMin=lerp(lower->speedMaxMMin,upper->speedMaxMMin);
    out.speedMMin=lerp(lower->speedMMin,upper->speedMMin);
    out.pierceSeconds=lerp(lower->pierceSeconds,upper->pierceSeconds);
    out.interpolated=true;
    out.outOfRange=false;
    out.note="Интерполировано между двумя справочными точками.";
    return out;
  }

  auto out=*nearest;
  out.interpolated=false;
  out.outOfRange=(lower==nullptr||upper==nullptr);
  if(out.outOfRange)
    out.note="Толщина вне диапазона базы: используется ближайшая точка; требуется ручная проверка.";
  return out;
}

}
