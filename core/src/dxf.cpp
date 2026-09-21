#include "sheetnest/dxf.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace sheetnest {
namespace {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kEps = 1e-8;

struct Group {
    int code{};
    std::string value;
};

struct Entity {
    std::string type;
    std::vector<Group> groups;
};

struct Segment {
    Polygon points;
    Point start{};
    Point end{};
    std::string layer;
    bool used{false};
};

struct Loop {
    Polygon polygon;
    std::string source;
    std::string layer;
    double area{};
};

double number(const std::string& v) {
    try { return std::stod(v); } catch (...) { return 0.0; }
}

int integer(const std::string& v) {
    try { return std::stoi(v); } catch (...) { return 0; }
}

std::vector<Group> readGroups(const std::string& text) {
    std::istringstream in(text);
    std::vector<Group> groups;
    std::string codeLine;
    std::string valueLine;

    while (std::getline(in, codeLine) && std::getline(in, valueLine)) {
        if (!codeLine.empty() && codeLine.back() == '\r') codeLine.pop_back();
        if (!valueLine.empty() && valueLine.back() == '\r') valueLine.pop_back();
        try {
            groups.push_back({std::stoi(codeLine), valueLine});
        } catch (...) {
            // Ignore malformed pairs; the resulting missing entity is diagnosed.
        }
    }
    return groups;
}

std::vector<Entity> readEntities(const std::string& text, std::size_t& count) {
    const auto groups = readGroups(text);
    std::vector<Entity> entities;
    bool inEntities = false;

    for (std::size_t i = 0; i < groups.size();) {
        if (groups[i].code != 0) {
            ++i;
            continue;
        }

        if (groups[i].value == "SECTION") {
            std::string name;
            std::size_t j = i + 1;
            while (j < groups.size() && groups[j].code != 0) {
                if (groups[j].code == 2) {
                    name = groups[j].value;
                    break;
                }
                ++j;
            }
            inEntities = (name == "ENTITIES");
            i = j;
            continue;
        }

        if (groups[i].value == "ENDSEC") {
            inEntities = false;
            ++i;
            continue;
        }

        if (!inEntities) {
            ++i;
            continue;
        }

        Entity e;
        e.type = groups[i].value;
        ++count;
        ++i;

        while (i < groups.size() && groups[i].code != 0) {
            e.groups.push_back(groups[i++]);
        }
        entities.push_back(std::move(e));
    }

    return entities;
}

double signedArea(const Polygon& p) {
    if (p.size() < 3) return 0.0;
    double a = 0.0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const auto& u = p[i];
        const auto& v = p[(i + 1) % p.size()];
        a += u.x * v.y - v.x * u.y;
    }
    return 0.5 * a;
}

bool samePoint(const Point& a, const Point& b, double tol = 1e-6) {
    return std::hypot(a.x - b.x, a.y - b.y) <= tol;
}

void appendUnique(Polygon& p, Point v) {
    if (p.empty() || !samePoint(p.back(), v)) p.push_back(v);
}

double groupValue(const Entity& e, int code, double fallback = 0.0) {
    for (const auto& g : e.groups) {
        if (g.code == code) return number(g.value);
    }
    return fallback;
}

int groupInt(const Entity& e, int code, int fallback = 0) {
    for (const auto& g : e.groups) {
        if (g.code == code) return integer(g.value);
    }
    return fallback;
}

std::string groupString(
    const Entity& e,
    int code,
    const std::string& fallback = {}
) {
    for (const auto& g : e.groups) {
        if (g.code == code) return g.value;
    }
    return fallback;
}

Point groupPoint(const Entity& e, int xCode, int yCode, Point fallback = {}) {
    Point p = fallback;
    bool xFound = false;
    bool yFound = false;
    for (const auto& g : e.groups) {
        if (g.code == xCode) {
            p.x = number(g.value);
            xFound = true;
        } else if (g.code == yCode) {
            p.y = number(g.value);
            yFound = true;
        }
    }
    return (xFound && yFound) ? p : fallback;
}

void appendArc(
    Polygon& out,
    Point center,
    double radius,
    double start,
    double sweep,
    double tolerance
) {
    if (radius <= kEps || std::abs(sweep) <= kEps) return;

    const double tol = std::max(0.01, std::min(std::abs(tolerance), radius * 0.5));
    double step = kTwoPi;
    if (tol < radius) {
        step = 2.0 * std::acos(std::clamp(1.0 - tol / radius, -1.0, 1.0));
    }
    step = std::max(step, kPi / 1800.0);

    const int segments = std::max(
        2,
        static_cast<int>(std::ceil(std::abs(sweep) / step))
    );

    for (int i = 0; i <= segments; ++i) {
        const double t = start + sweep * static_cast<double>(i) / segments;
        appendUnique(out, {
            center.x + radius * std::cos(t),
            center.y + radius * std::sin(t)
        });
    }
}

Polygon circle(Point center, double radius, double tolerance) {
    Polygon p;
    appendArc(p, center, radius, 0.0, kTwoPi, tolerance);
    if (!p.empty() && samePoint(p.front(), p.back())) p.pop_back();
    return p;
}

Polygon bulgeEdge(Point a, Point b, double bulge, double tolerance) {
    Polygon out;
    appendUnique(out, a);

    if (std::abs(bulge) <= kEps || samePoint(a, b)) {
        appendUnique(out, b);
        return out;
    }

    const double chord = std::hypot(b.x - a.x, b.y - a.y);
    const double sweep = 4.0 * std::atan(bulge);
    const double half = std::abs(sweep) * 0.5;
    const double sinHalf = std::sin(half);
    if (chord <= kEps || std::abs(sinHalf) <= kEps) {
        appendUnique(out, b);
        return out;
    }

    const double radius = chord / (2.0 * sinHalf);
    const double offset = chord / (2.0 * std::tan(half));
    const Point mid{(a.x + b.x) * 0.5, (a.y + b.y) * 0.5};
    const double inv = 1.0 / chord;
    const Point left{-(b.y - a.y) * inv, (b.x - a.x) * inv};
    const double sign = sweep >= 0.0 ? 1.0 : -1.0;
    const Point center{
        mid.x - sign * left.x * offset,
        mid.y - sign * left.y * offset
    };
    const double start = std::atan2(a.y - center.y, a.x - center.x);

    appendArc(out, center, radius, start, sweep, tolerance);
    appendUnique(out, b);
    return out;
}

struct Vertex {
    Point point{};
    double bulge{};
};

Polygon polylineGeometry(
    const std::vector<Vertex>& vertices,
    bool closed,
    double tolerance
) {
    if (vertices.size() < 2) return {};

    Polygon result;
    const std::size_t edgeCount = closed ? vertices.size() : vertices.size() - 1;

    for (std::size_t i = 0; i < edgeCount; ++i) {
        const auto& a = vertices[i];
        const auto& b = vertices[(i + 1) % vertices.size()];
        const auto edge = bulgeEdge(a.point, b.point, a.bulge, tolerance);
        for (std::size_t j = 0; j < edge.size(); ++j) {
            if (i > 0 && j == 0) continue;
            appendUnique(result, edge[j]);
        }
    }

    if (closed && !result.empty() && samePoint(result.front(), result.back())) {
        result.pop_back();
    }
    return result;
}

Polygon lwPolyline(const Entity& entity, bool& closed, double tolerance) {
    std::vector<Vertex> vertices;
    Vertex current{};
    bool haveX = false;
    bool haveY = false;

    for (const auto& g : entity.groups) {
        if (g.code == 10) {
            if (haveX && haveY) vertices.push_back(current);
            current = {};
            current.point.x = number(g.value);
            haveX = true;
            haveY = false;
        } else if (g.code == 20 && haveX) {
            current.point.y = number(g.value);
            haveY = true;
        } else if (g.code == 42 && haveX) {
            current.bulge = number(g.value);
        }
    }
    if (haveX && haveY) vertices.push_back(current);

    closed = (groupInt(entity, 70) & 1) != 0;
    return polylineGeometry(vertices, closed, tolerance);
}

bool contains(const Polygon& p, const Point& q) {
    return pointInPolygon(q, p);
}

void normalize(Polygon& p, bool outer) {
    const double a = signedArea(p);
    if ((outer && a < 0.0) || (!outer && a > 0.0)) {
        std::reverse(p.begin(), p.end());
    }
}

std::vector<DxfContour> classifyLoops(std::vector<Loop> loops) {
    std::vector<DxfContour> result;
    if (loops.empty()) return result;

    const std::size_t n = loops.size();
    std::vector<int> depth(n, 0);

    for (std::size_t i = 0; i < n; ++i) {
        const Point probe = loops[i].polygon.front();
        for (std::size_t j = 0; j < n; ++j) {
            if (i == j) continue;
            if (std::abs(loops[j].area) <= std::abs(loops[i].area)) continue;
            if (contains(loops[j].polygon, probe)) ++depth[i];
        }
    }

    std::vector<int> contourForLoop(n, -1);

    for (std::size_t i = 0; i < n; ++i) {
        if ((depth[i] & 1) != 0) continue;

        auto outer = loops[i].polygon;
        normalize(outer, true);

        contourForLoop[i] = static_cast<int>(result.size());
        result.push_back({
            std::move(outer),
            {},
            loops[i].source + "-" + std::to_string(result.size() + 1),
            loops[i].layer
        });
    }

    for (std::size_t i = 0; i < n; ++i) {
        if ((depth[i] & 1) == 0) continue;

        const Point probe = loops[i].polygon.front();
        std::size_t parent = n;
        double parentArea = std::numeric_limits<double>::infinity();

        for (std::size_t j = 0; j < n; ++j) {
            if ((depth[j] & 1) != 0) continue;
            if (std::abs(loops[j].area) <= std::abs(loops[i].area)) continue;
            if (!contains(loops[j].polygon, probe)) continue;

            const double area = std::abs(loops[j].area);
            if (area < parentArea) {
                parentArea = area;
                parent = j;
            }
        }

        if (parent == n || contourForLoop[parent] < 0) continue;

        auto hole = loops[i].polygon;
        normalize(hole, false);
        result[contourForLoop[parent]].holes.push_back(std::move(hole));
    }

    return result;
}

std::vector<Loop> connectSegments(
    std::vector<Segment> segments,
    std::vector<DxfDiagnostic>& diagnostics
) {
    std::vector<Loop> loops;

    for (std::size_t startIndex = 0; startIndex < segments.size(); ++startIndex) {
        if (segments[startIndex].used) continue;

        segments[startIndex].used = true;
        Polygon polygon = segments[startIndex].points;
        const Point initial = segments[startIndex].start;
        Point current = segments[startIndex].end;
        bool closed = false;

        for (std::size_t guard = 0; guard <= segments.size(); ++guard) {
            if (samePoint(current, initial)) {
                closed = true;
                break;
            }

            std::size_t next = segments.size();
            bool reversed = false;

            const auto& currentSegment = segments[startIndex];
            for (std::size_t i = 0; i < segments.size(); ++i) {
                if (segments[i].used) continue;
                if (segments[i].layer != currentSegment.layer) continue;
                if (samePoint(segments[i].start, current)) {
                    next = i;
                    reversed = false;
                    break;
                }
                if (samePoint(segments[i].end, current)) {
                    next = i;
                    reversed = true;
                    break;
                }
            }

            if (next == segments.size()) break;

            auto& s = segments[next];
            s.used = true;

            if (!reversed) {
                for (std::size_t i = 1; i < s.points.size(); ++i) {
                    appendUnique(polygon, s.points[i]);
                }
                current = s.end;
            } else {
                for (std::size_t i = s.points.size(); i-- > 1;) {
                    appendUnique(polygon, s.points[i - 1]);
                }
                current = s.start;
            }
        }

        if (closed && polygon.size() >= 3) {
            loops.push_back({
                std::move(polygon),
                "LINE/ARC",
                segments[startIndex].layer,
                signedArea(polygon)
            });
        } else {
            diagnostics.push_back({
                DxfSeverity::Warning,
                "DXF/CONNECT",
                "Незамкнутая цепочка LINE/ARC пропущена."
            });
        }
    }

    return loops;
}

std::vector<Segment> collectSegments(
    const std::vector<Entity>& entities,
    std::vector<Loop>& directLoops,
    std::vector<DxfDiagnostic>& diagnostics,
    double tolerance
) {
    std::vector<Segment> segments;

    for (std::size_t i = 0; i < entities.size();) {
        const auto& e = entities[i];

        if (e.type == "LINE") {
            const auto a = groupPoint(e, 10, 20);
            const auto b = groupPoint(e, 11, 21);
            if (!samePoint(a, b)) {
                segments.push_back({{a, b}, a, b, groupString(e, 8, "0"), false});
            }
            ++i;
            continue;
        }

        if (e.type == "ARC") {
            const auto center = groupPoint(e, 10, 20);
            const double radius = groupValue(e, 40);
            const double startDeg = groupValue(e, 50);
            const double endDeg = groupValue(e, 51);
            double sweepDeg = endDeg - startDeg;
            if (sweepDeg <= 0.0) sweepDeg += 360.0;

            Polygon arc;
            appendArc(
                arc,
                center,
                radius,
                startDeg * kPi / 180.0,
                sweepDeg * kPi / 180.0,
                tolerance
            );

            if (arc.size() >= 2) {
                segments.push_back({arc, arc.front(), arc.back(), groupString(e, 8, "0"), false});
            }
            ++i;
            continue;
        }

        if (e.type == "CIRCLE") {
            auto p = circle(groupPoint(e, 10, 20), groupValue(e, 40), tolerance);
            if (p.size() >= 3) {
                const double area = signedArea(p);
                directLoops.push_back({
                    std::move(p),
                    "CIRCLE",
                    groupString(e, 8, "0"),
                    area
                });
            }
            ++i;
            continue;
        }

        if (e.type == "LWPOLYLINE") {
            bool closed = false;
            auto p = lwPolyline(e, closed, tolerance);
            if (closed && p.size() >= 3) {
                const double area = signedArea(p);
                directLoops.push_back({
                    std::move(p),
                    "LWPOLYLINE",
                    groupString(e, 8, "0"),
                    area
                });
            } else if (p.size() >= 2) {
                diagnostics.push_back({
                    DxfSeverity::Warning,
                    "DXF/LWPOLYLINE",
                    "Открытая LWPOLYLINE пропущена: контур не замкнут."
                });
            }
            ++i;
            continue;
        }

        if (e.type == "POLYLINE") {
            std::vector<Vertex> vertices;
            bool closed = false;

            ++i;
            while (i < entities.size() && entities[i].type != "SEQEND") {
                if (entities[i].type == "VERTEX") {
                    vertices.push_back({
                        groupPoint(entities[i], 10, 20),
                        groupValue(entities[i], 42, 0.0)
                    });
                }
                ++i;
            }
            if (i < entities.size() && entities[i].type == "SEQEND") ++i;

            closed = (groupInt(e, 70) & 1) != 0;
            auto p = polylineGeometry(vertices, closed, tolerance);

            if (closed && p.size() >= 3) {
                const double area = signedArea(p);
                directLoops.push_back({
                    std::move(p),
                    "POLYLINE",
                    groupString(e, 8, "0"),
                    area
                });
            } else if (p.size() >= 2) {
                diagnostics.push_back({
                    DxfSeverity::Warning,
                    "DXF/POLYLINE",
                    "Открытая POLYLINE пропущена: контур не замкнут."
                });
            }
            continue;
        }

        ++i;
    }

    return segments;
}

} // namespace

bool DxfDocument::valid() const {
    return !contours.empty();
}

DxfDocument importDxf(const std::string& text, double arcToleranceMm) {
    DxfDocument document;

    if (text.empty()) {
        document.diagnostics.push_back({
            DxfSeverity::Error,
            "DXF/INPUT",
            "DXF-текст пуст."
        });
        return document;
    }

    const auto entities = readEntities(text, document.entitiesRead);
    if (entities.empty()) {
        document.diagnostics.push_back({
            DxfSeverity::Error,
            "DXF/ENTITIES",
            "В DXF не найден раздел ENTITIES с объектами."
        });
        return document;
    }

    std::vector<Loop> loops;
    auto segments = collectSegments(
        entities,
        loops,
        document.diagnostics,
        std::max(0.01, arcToleranceMm)
    );

    auto connected = connectSegments(
        std::move(segments),
        document.diagnostics
    );
    loops.insert(
        loops.end(),
        std::make_move_iterator(connected.begin()),
        std::make_move_iterator(connected.end())
    );

    document.closedLoopsFound = loops.size();
    document.contours = classifyLoops(std::move(loops));

    if (document.contours.empty()) {
        document.diagnostics.push_back({
            DxfSeverity::Error,
            "DXF/GEOMETRY",
            "Не найден ни один валидный замкнутый контур. Проверьте замыкание LINE/ARC/POLYLINE."
        });
    } else {
        document.diagnostics.push_back({
            DxfSeverity::Info,
            "DXF/GEOMETRY",
            "Геометрия преобразована в внешние контуры и отверстия."
        });
    }

    return document;
}

} // namespace sheetnest
