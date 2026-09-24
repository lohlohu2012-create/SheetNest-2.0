#include "sheetnest/dxf.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace sheetnest {
namespace {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kEps = 1e-9;
constexpr double kAreaEps = 1e-7;

struct Group {
    int code{};
    std::string value;
    std::size_t line{};
};

struct Entity {
    std::string type;
    std::vector<Group> groups;
    std::vector<Entity> children;
    std::size_t index{};
};

struct Segment {
    Polygon points;
    Point start{};
    Point end{};
    Point startTangent{};
    Point endTangent{};
    std::string layer;
    std::string source;
    std::size_t startNode{};
    std::size_t endNode{};
    bool used{false};
};

struct TraversedEdge {
    std::size_t segment{};
    bool forward{true};
};

struct Loop {
    Polygon polygon;
    std::string source;
    std::string layer;
    double area{};
    std::vector<std::size_t> segments;
};

void diagnostic(
    std::vector<DxfDiagnostic>& out,
    DxfSeverity severity,
    const std::string& stage,
    const std::string& message,
    const Entity* entity = nullptr
) {
    DxfDiagnostic d;
    d.severity = severity;
    d.stage = stage;
    d.message = message;
    if (entity) {
        d.entityIndex = entity->index;
        d.entityType = entity->type;
        for (const auto& g : entity->groups) {
            if (g.code == 8) {
                d.layer = g.value;
                break;
            }
        }
    }
    out.push_back(std::move(d));
}

std::string trimCode(std::string value) {
    while (!value.empty() && (value.back() == ' ' || value.back() == '\t' || value.back() == '\r')) {
        value.pop_back();
    }
    std::size_t first = 0;
    while (first < value.size() && (value[first] == ' ' || value[first] == '\t')) ++first;
    return value.substr(first);
}

std::optional<int> parseIntExact(const std::string& value) {
    try {
        const std::string normalized = trimCode(value);
        if (normalized.empty()) return std::nullopt;
        std::size_t used{};
        const int n = std::stoi(normalized, &used);
        if (used != normalized.size()) return std::nullopt;
        return n;
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<double> parseDoubleExact(const std::string& value) {
    try {
        const std::string normalized = trimCode(value);
        std::size_t used{};
        const double n = std::stod(normalized, &used);
        if (used != normalized.size() || !std::isfinite(n)) return std::nullopt;
        return n;
    } catch (...) {
        return std::nullopt;
    }
}

std::string groupString(
    const Entity& entity,
    int code,
    const std::string& fallback = {}
) {
    for (const auto& g : entity.groups) {
        if (g.code == code) return g.value;
    }
    return fallback;
}

std::optional<double> groupNumber(const Entity& entity, int code) {
    for (const auto& g : entity.groups) {
        if (g.code == code) return parseDoubleExact(g.value);
    }
    return std::nullopt;
}

int groupInt(const Entity& entity, int code, int fallback = 0) {
    for (const auto& g : entity.groups) {
        if (g.code == code) {
            const auto value = parseIntExact(g.value);
            return value.value_or(fallback);
        }
    }
    return fallback;
}

bool requireNumber(
    const Entity& entity,
    int code,
    double& value,
    std::vector<DxfDiagnostic>& diagnostics,
    const char* field
) {
    const auto parsed = groupNumber(entity, code);
    if (!parsed.has_value()) {
        diagnostic(
            diagnostics,
            DxfSeverity::Error,
            "DXF/VALIDATE",
            std::string("Обязательное поле ") + field + " отсутствует или содержит некорректное число.",
            &entity
        );
        return false;
    }
    value = *parsed;
    return true;
}

bool requirePoint(
    const Entity& entity,
    int xCode,
    int yCode,
    Point& point,
    std::vector<DxfDiagnostic>& diagnostics,
    const char* field
) {
    return requireNumber(entity, xCode, point.x, diagnostics, field) &&
           requireNumber(entity, yCode, point.y, diagnostics, field);
}

std::vector<Group> readGroups(
    const std::string& text,
    std::vector<DxfDiagnostic>& diagnostics
) {
    std::istringstream input(text);
    std::vector<Group> groups;

    std::size_t line = 0;
    std::string codeLine;
    std::string valueLine;

    while (std::getline(input, codeLine)) {
        ++line;
        if (!std::getline(input, valueLine)) {
            diagnostic(
                diagnostics,
                DxfSeverity::Error,
                "DXF/PAIRS",
                "DXF обрывается на строке с group code без соответствующего значения."
            );
            break;
        }
        ++line;

        if (!codeLine.empty() && codeLine.back() == '\r') codeLine.pop_back();
        if (!valueLine.empty() && valueLine.back() == '\r') valueLine.pop_back();

        const auto code = parseIntExact(codeLine);
        if (!code.has_value()) {
            diagnostic(
                diagnostics,
                DxfSeverity::Error,
                "DXF/PAIRS",
                "Некорректный group code на строке " + std::to_string(line - 1) + "."
            );
            continue;
        }

        groups.push_back({*code, valueLine, line - 1});
    }

    return groups;
}

std::vector<Entity> readEntities(
    const std::vector<Group>& groups,
    std::vector<DxfDiagnostic>& diagnostics,
    std::size_t& entitiesRead,
    bool& foundEntitiesSection
) {
    std::vector<Entity> entities;
    bool inEntities = false;
    std::size_t topLevelIndex = 0;

    for (std::size_t i = 0; i < groups.size();) {
        if (groups[i].code != 0) {
            ++i;
            continue;
        }

        const std::string marker = trimCode(groups[i].value);

        if (marker == "SECTION") {
            std::string sectionName;
            std::size_t j = i + 1;
            while (j < groups.size() && groups[j].code != 0) {
                if (groups[j].code == 2) {
                    sectionName = trimCode(groups[j].value);
                    break;
                }
                ++j;
            }
            inEntities = (sectionName == "ENTITIES");
            foundEntitiesSection = foundEntitiesSection || inEntities;
            i = j;
            continue;
        }

        if (marker == "ENDSEC") {
            inEntities = false;
            ++i;
            continue;
        }

        if (!inEntities) {
            ++i;
            continue;
        }

        if (marker == "EOF") break;

        Entity entity;
        entity.type = marker;
        entity.index = ++topLevelIndex;
        ++entitiesRead;
        ++i;

        while (i < groups.size() && groups[i].code != 0) {
            entity.groups.push_back(groups[i]);
            ++i;
        }

        if (entity.type == "POLYLINE") {
            bool sawSeqend = false;
            while (i < groups.size() && groups[i].code == 0) {
                const std::string childType = trimCode(groups[i].value);
                if (childType == "SEQEND") {
                    Entity seqend;
                    seqend.type = childType;
                    seqend.index = entity.index;
                    ++i;
                    sawSeqend = true;
                    break;
                }

                if (childType != "VERTEX") break;

                Entity vertex;
                vertex.type = childType;
                vertex.index = entity.index;
                ++i;
                while (i < groups.size() && groups[i].code != 0) {
                    vertex.groups.push_back(groups[i]);
                    ++i;
                }
                entity.children.push_back(std::move(vertex));
            }

            if (!sawSeqend) {
                diagnostic(
                    diagnostics,
                    DxfSeverity::Warning,
                    "DXF/POLYLINE",
                    "POLYLINE не содержит корректного SEQEND. Вершины обработаны до следующей сущности.",
                    &entity
                );
            }
        }

        entities.push_back(std::move(entity));
    }

    return entities;
}

double signedArea(const Polygon& polygon) {
    if (polygon.size() < 3) return 0.0;

    double area = 0.0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const auto& a = polygon[i];
        const auto& b = polygon[(i + 1) % polygon.size()];
        area += a.x * b.y - b.x * a.y;
    }
    return area * 0.5;
}

bool samePoint(const Point& a, const Point& b, double tolerance) {
    return std::hypot(a.x - b.x, a.y - b.y) <= tolerance;
}

void appendUnique(Polygon& polygon, Point point, double tolerance = 1e-8) {
    if (polygon.empty() || !samePoint(polygon.back(), point, tolerance)) {
        polygon.push_back(point);
    }
}

Point normalizeVector(Point v) {
    const double length = std::hypot(v.x, v.y);
    if (length <= kEps) return {};
    return {v.x / length, v.y / length};
}

double dot(Point a, Point b) {
    return a.x * b.x + a.y * b.y;
}

bool pointOnSegment(Point point, Point a, Point b, double tolerance) {
    const double cross =
        (b.x - a.x) * (point.y - a.y) -
        (b.y - a.y) * (point.x - a.x);
    if (std::abs(cross) > tolerance * std::max(1.0, std::hypot(b.x - a.x, b.y - a.y))) {
        return false;
    }
    return point.x >= std::min(a.x, b.x) - tolerance &&
           point.x <= std::max(a.x, b.x) + tolerance &&
           point.y >= std::min(a.y, b.y) - tolerance &&
           point.y <= std::max(a.y, b.y) + tolerance;
}

bool pointInPolygonInclusive(
    const Point& point,
    const Polygon& polygon,
    double tolerance
) {
    if (polygon.size() < 3) return false;

    bool inside = false;
    for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        const Point a = polygon[i];
        const Point b = polygon[j];

        if (pointOnSegment(point, a, b, tolerance)) return true;

        if ((a.y > point.y) != (b.y > point.y)) {
            const double x =
                (b.x - a.x) * (point.y - a.y) /
                    ((b.y - a.y) + std::numeric_limits<double>::epsilon()) +
                a.x;
            if (point.x < x) inside = !inside;
        }
    }
    return inside;
}

double pointSegmentDistance(Point p, Point a, Point b) {
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double len2 = vx * vx + vy * vy;
    if (len2 <= kEps) return std::hypot(p.x - a.x, p.y - a.y);

    double t = ((p.x - a.x) * vx + (p.y - a.y) * vy) / len2;
    t = std::clamp(t, 0.0, 1.0);
    const Point q{a.x + t * vx, a.y + t * vy};
    return std::hypot(p.x - q.x, p.y - q.y);
}

bool segmentsIntersect(Point a, Point b, Point c, Point d, double tolerance) {
    if (pointSegmentDistance(a, c, d) <= tolerance ||
        pointSegmentDistance(b, c, d) <= tolerance ||
        pointSegmentDistance(c, a, b) <= tolerance ||
        pointSegmentDistance(d, a, b) <= tolerance) {
        return true;
    }

    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double cdx = d.x - c.x;
    const double cdy = d.y - c.y;

    const double p =
        abx * (c.y - a.y) -
        aby * (c.x - a.x);
    const double q =
        abx * (d.y - a.y) -
        aby * (d.x - a.x);
    const double r =
        cdx * (a.y - c.y) -
        cdy * (a.x - c.x);
    const double s =
        cdx * (b.y - c.y) -
        cdy * (b.x - c.x);

    return ((p > tolerance && q < -tolerance) || (p < -tolerance && q > tolerance)) &&
           ((r > tolerance && s < -tolerance) || (r < -tolerance && s > tolerance));
}

bool selfIntersects(const Polygon& polygon, double tolerance) {
    if (polygon.size() < 4) return false;

    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const Point a0 = polygon[i];
        const Point a1 = polygon[(i + 1) % polygon.size()];

        for (std::size_t j = i + 1; j < polygon.size(); ++j) {
            if (j == i || (j + 1) % polygon.size() == i || (i + 1) % polygon.size() == j) {
                continue;
            }

            const Point b0 = polygon[j];
            const Point b1 = polygon[(j + 1) % polygon.size()];
            if (segmentsIntersect(a0, a1, b0, b1, tolerance)) return true;
        }
    }

    return false;
}

Polygon sampleArc(
    Point center,
    double radius,
    double startRadians,
    double sweepRadians,
    double tolerance
) {
    Polygon polygon;
    if (radius <= kEps || std::abs(sweepRadians) <= kEps) return polygon;

    const double tol = std::max(
        0.001,
        std::min(std::abs(tolerance), radius * 0.5)
    );

    double step = kTwoPi;
    if (tol < radius) {
        step = 2.0 * std::acos(
            std::clamp(1.0 - tol / radius, -1.0, 1.0)
        );
    }

    step = std::max(step, kPi / 3600.0);

    const int segments = std::max(
        2,
        static_cast<int>(std::ceil(std::abs(sweepRadians) / step))
    );

    auto pointAt = [&](double angle) {
        return Point{
            center.x + radius * std::cos(angle),
            center.y + radius * std::sin(angle)
        };
    };

    polygon.reserve(static_cast<std::size_t>(segments) + 1);
    polygon.push_back(pointAt(startRadians));

    for (int i = 1; i < segments; ++i) {
        const double t =
            startRadians +
            sweepRadians * static_cast<double>(i) / static_cast<double>(segments);
        polygon.push_back(pointAt(t));
    }

    polygon.push_back(pointAt(startRadians + sweepRadians));
    return polygon;
}

Polygon circle(Point center, double radius, double tolerance) {
    auto polygon = sampleArc(center, radius, 0.0, kTwoPi, tolerance);
    if (!polygon.empty() && samePoint(polygon.front(), polygon.back(), 1e-8)) {
        polygon.pop_back();
    }
    return polygon;
}

Polygon bulgeEdge(
    Point a,
    Point b,
    double bulge,
    double tolerance
) {
    Polygon result;
    appendUnique(result, a);

    if (std::abs(bulge) <= kEps) {
        appendUnique(result, b);
        return result;
    }

    const double chord = std::hypot(b.x - a.x, b.y - a.y);
    if (chord <= kEps) {
        return result;
    }

    // DXF bulge = tan(includedAngle / 4). Positive bulge is a CCW arc
    // from a to b. This center formula works for arcs below and above 180°.
    const double sweep = 4.0 * std::atan(bulge);
    const double offset =
        chord * (1.0 - bulge * bulge) / (4.0 * bulge);

    const Point midpoint{
        (a.x + b.x) * 0.5,
        (a.y + b.y) * 0.5
    };

    const double invChord = 1.0 / chord;
    const Point leftNormal{
        -(b.y - a.y) * invChord,
        (b.x - a.x) * invChord
    };

    const Point center{
        midpoint.x + leftNormal.x * offset,
        midpoint.y + leftNormal.y * offset
    };

    const double startAngle =
        std::atan2(a.y - center.y, a.x - center.x);

    result = sampleArc(
        center,
        std::hypot(a.x - center.x, a.y - center.y),
        startAngle,
        sweep,
        tolerance
    );

    if (result.empty()) {
        result.push_back(a);
    } else {
        result.front() = a;
    }
    appendUnique(result, b);
    return result;
}

struct Vertex {
    Point point{};
    double bulge{};
    bool valid{false};
};

std::vector<Vertex> readPolylineVertices(
    const Entity& entity,
    const std::vector<Entity>& vertices,
    std::vector<DxfDiagnostic>& diagnostics,
    double tolerance
) {
    std::vector<Vertex> result;

    if (vertices.empty()) {
        diagnostic(
            diagnostics,
            DxfSeverity::Error,
            "DXF/" + entity.type,
            "В полилинии отсутствуют VERTEX.",
            &entity
        );
        return result;
    }

    for (const auto& vertex : vertices) {
        Vertex item;
        if (!requirePoint(
                vertex,
                10,
                20,
                item.point,
                diagnostics,
                "10/20"
            )) {
            diagnostic(
                diagnostics,
                DxfSeverity::Error,
                "DXF/" + entity.type,
                "VERTEX пропущен из-за отсутствующих координат 10/20.",
                &entity
            );
            continue;
        }

        const auto bulge = groupNumber(vertex, 42);
        item.bulge = bulge.value_or(0.0);
        item.valid = true;
        result.push_back(item);
    }

    if (result.size() >= 2) {
        for (std::size_t i = 1; i < result.size(); ++i) {
            if (samePoint(result[i - 1].point, result[i].point, tolerance)) {
                diagnostic(
                    diagnostics,
                    DxfSeverity::Warning,
                    "DXF/" + entity.type,
                    "Найдены совпадающие соседние вершины; дубликат будет удалён."
                );
            }
        }
    }

    if (result.size() < vertices.size()) {
        diagnostic(
            diagnostics,
            DxfSeverity::Error,
            "DXF/" + entity.type,
            "После проверки POLYLINE осталось недостаточно валидных VERTEX. Сущность отклонена.",
            &entity
        );
        result.clear();
    }

    return result;
}

Polygon polylineGeometry(
    const std::vector<Vertex>& vertices,
    bool closed,
    double tolerance
) {
    if (vertices.size() < 2) return {};

    Polygon result;
    const std::size_t edgeCount =
        closed ? vertices.size() : vertices.size() - 1;

    for (std::size_t i = 0; i < edgeCount; ++i) {
        const auto& a = vertices[i];
        const auto& b = vertices[(i + 1) % vertices.size()];
        const Polygon edge = bulgeEdge(
            a.point,
            b.point,
            a.bulge,
            tolerance
        );

        for (std::size_t j = 0; j < edge.size(); ++j) {
            if (i != 0 && j == 0) continue;
            appendUnique(result, edge[j], tolerance * 0.05);
        }
    }

    if (closed &&
        !result.empty() &&
        samePoint(result.front(), result.back(), tolerance * 0.05)) {
        result.pop_back();
    }

    return result;
}

Point startTangent(const Polygon& polygon) {
    if (polygon.size() < 2) return {};
    return normalizeVector({
        polygon[1].x - polygon[0].x,
        polygon[1].y - polygon[0].y
    });
}

Point endTangent(const Polygon& polygon) {
    if (polygon.size() < 2) return {};
    const Point a = polygon[polygon.size() - 2];
    const Point b = polygon.back();
    return normalizeVector({
        b.x - a.x,
        b.y - a.y
    });
}

struct GridKey {
    long long x{};
    long long y{};
    std::string layer;

    bool operator==(const GridKey& other) const {
        return x == other.x && y == other.y && layer == other.layer;
    }
};

struct GridKeyHash {
    std::size_t operator()(const GridKey& key) const {
        std::size_t h1 = std::hash<long long>{}(key.x);
        std::size_t h2 = std::hash<long long>{}(key.y);
        std::size_t h3 = std::hash<std::string>{}(key.layer);
        return h1 ^ (h2 << 1) ^ (h3 << 7);
    }
};

class EndpointClusterer {
public:
    explicit EndpointClusterer(double tolerance)
        : tolerance_(std::max(1e-6, tolerance)) {}

    std::size_t node(Point point, const std::string& layer) {
        const long long gx = static_cast<long long>(std::llround(point.x / tolerance_));
        const long long gy = static_cast<long long>(std::llround(point.y / tolerance_));

        for (long long dx = -1; dx <= 1; ++dx) {
            for (long long dy = -1; dy <= 1; ++dy) {
                GridKey candidate{gx + dx, gy + dy, layer};
                const auto it = buckets_.find(candidate);
                if (it == buckets_.end()) continue;

                for (const std::size_t index : it->second) {
                    if (samePoint(points_[index], point, tolerance_)) {
                        return index;
                    }
                }
            }
        }

        const std::size_t index = points_.size();
        points_.push_back(point);
        buckets_[{gx, gy, layer}].push_back(index);
        return index;
    }

    Point point(std::size_t index) const {
        return points_[index];
    }

private:
    double tolerance_{};
    std::vector<Point> points_;
    std::unordered_map<GridKey, std::vector<std::size_t>, GridKeyHash> buckets_;
};

bool parseEntityAsSegment(
    const Entity& entity,
    Polygon polygon,
    std::vector<Segment>& segments,
    const std::string& source
) {
    if (polygon.size() < 2) return false;

    Segment segment;
    segment.points = std::move(polygon);
    segment.start = segment.points.front();
    segment.end = segment.points.back();
    segment.startTangent = startTangent(segment.points);
    segment.endTangent = endTangent(segment.points);
    segment.layer = groupString(entity, 8, "0");
    segment.source = source;
    segments.push_back(std::move(segment));
    return true;
}

bool parseEntityLine(
    const Entity& entity,
    std::vector<Segment>& segments,
    std::vector<DxfDiagnostic>& diagnostics,
    std::size_t& supported,
    std::size_t& malformed
) {
    Point a;
    Point b;
    if (!requirePoint(entity, 10, 20, a, diagnostics, "10/20") ||
        !requirePoint(entity, 11, 21, b, diagnostics, "11/21")) {
        ++malformed;
        return false;
    }

    if (samePoint(a, b, 1e-8)) {
        diagnostic(
            diagnostics,
            DxfSeverity::Error,
            "DXF/LINE",
            "LINE имеет нулевую длину.",
            &entity
        );
        ++malformed;
        return false;
    }

    parseEntityAsSegment(
        entity,
        {a, b},
        segments,
        "LINE#" + std::to_string(entity.index)
    );
    ++supported;
    return true;
}

bool parseEntityArc(
    const Entity& entity,
    std::vector<Segment>& segments,
    std::vector<DxfDiagnostic>& diagnostics,
    std::size_t& supported,
    std::size_t& malformed,
    double tolerance
) {
    Point center;
    double radius{};
    double startDegrees{};
    double endDegrees{};

    if (!requirePoint(entity, 10, 20, center, diagnostics, "10/20") ||
        !requireNumber(entity, 40, radius, diagnostics, "40 radius") ||
        !requireNumber(entity, 50, startDegrees, diagnostics, "50 start angle") ||
        !requireNumber(entity, 51, endDegrees, diagnostics, "51 end angle")) {
        ++malformed;
        return false;
    }

    if (radius <= kEps) {
        diagnostic(
            diagnostics,
            DxfSeverity::Error,
            "DXF/ARC",
            "ARC имеет неположительный радиус.",
            &entity
        );
        ++malformed;
        return false;
    }

    double sweepDegrees = endDegrees - startDegrees;
    if (std::abs(sweepDegrees) <= 1e-9) {
        diagnostic(
            diagnostics,
            DxfSeverity::Error,
            "DXF/ARC",
            "ARC имеет одинаковые start/end angles и не содержит дугу.",
            &entity
        );
        ++malformed;
        return false;
    }
    if (sweepDegrees < 0.0) sweepDegrees += 360.0;

    if (sweepDegrees <= kEps || sweepDegrees > 360.0 + 1e-6) {
        diagnostic(
            diagnostics,
            DxfSeverity::Error,
            "DXF/ARC",
            "Некорректный диапазон углов ARC.",
            &entity
        );
        ++malformed;
        return false;
    }

    const double startRadians = startDegrees * kPi / 180.0;
    const double sweepRadians = sweepDegrees * kPi / 180.0;
    Polygon polygon = sampleArc(
        center,
        radius,
        startRadians,
        sweepRadians,
        tolerance
    );

    if (polygon.size() < 2) {
        diagnostic(
            diagnostics,
            DxfSeverity::Error,
            "DXF/ARC",
            "Не удалось аппроксимировать ARC.",
            &entity
        );
        ++malformed;
        return false;
    }

    polygon.front() = {
        center.x + radius * std::cos(startRadians),
        center.y + radius * std::sin(startRadians)
    };
    polygon.back() = {
        center.x + radius * std::cos(startRadians + sweepRadians),
        center.y + radius * std::sin(startRadians + sweepRadians)
    };

    parseEntityAsSegment(
        entity,
        std::move(polygon),
        segments,
        "ARC#" + std::to_string(entity.index)
    );
    ++supported;
    return true;
}

bool parseEntityCircle(
    const Entity& entity,
    std::vector<Loop>& loops,
    std::vector<DxfDiagnostic>& diagnostics,
    std::size_t& supported,
    std::size_t& malformed,
    double tolerance
) {
    Point center;
    double radius{};

    if (!requirePoint(entity, 10, 20, center, diagnostics, "10/20") ||
        !requireNumber(entity, 40, radius, diagnostics, "40 radius")) {
        ++malformed;
        return false;
    }

    if (radius <= kEps) {
        diagnostic(
            diagnostics,
            DxfSeverity::Error,
            "DXF/CIRCLE",
            "CIRCLE имеет неположительный радиус.",
            &entity
        );
        ++malformed;
        return false;
    }

    Polygon polygon = circle(center, radius, tolerance);
    if (polygon.size() < 8) {
        diagnostic(
            diagnostics,
            DxfSeverity::Error,
            "DXF/CIRCLE",
            "Не удалось построить геометрию CIRCLE.",
            &entity
        );
        ++malformed;
        return false;
    }

    loops.push_back({
        std::move(polygon),
        "CIRCLE#" + std::to_string(entity.index),
        groupString(entity, 8, "0"),
        0.0,
        {}
    });
    loops.back().area = signedArea(loops.back().polygon);
    ++supported;
    return true;
}

bool parseEntityLwPolyline(
    const Entity& entity,
    std::vector<Segment>& segments,
    std::vector<Loop>& loops,
    std::vector<DxfDiagnostic>& diagnostics,
    std::size_t& supported,
    std::size_t& malformed,
    double tolerance
) {
    std::vector<Vertex> vertices;
    Vertex current;
    bool haveX = false;
    bool haveY = false;
    bool malformedVertex = false;

    for (const auto& group : entity.groups) {
        if (group.code == 10) {
            if (haveX) {
                if (!haveY) malformedVertex = true;
                else vertices.push_back(current);
            }
            current = {};
            const auto x = parseDoubleExact(group.value);
            if (!x.has_value()) {
                malformedVertex = true;
                haveX = false;
                haveY = false;
                continue;
            }
            current.point.x = *x;
            haveX = true;
            haveY = false;
        } else if (group.code == 20) {
            if (!haveX) {
                malformedVertex = true;
                continue;
            }
            const auto y = parseDoubleExact(group.value);
            if (!y.has_value()) {
                malformedVertex = true;
                continue;
            }
            current.point.y = *y;
            haveY = true;
        } else if (group.code == 42) {
            if (!haveX) {
                malformedVertex = true;
                continue;
            }
            const auto b = parseDoubleExact(group.value);
            if (!b.has_value()) {
                malformedVertex = true;
                continue;
            }
            current.bulge = *b;
        }
    }

    if (haveX) {
        if (!haveY) malformedVertex = true;
        else vertices.push_back(current);
    }

    if (malformedVertex) {
        diagnostic(
            diagnostics,
            DxfSeverity::Error,
            "DXF/LWPOLYLINE",
            "Одна или несколько вершин LWPOLYLINE имеют некорректные 10/20 координаты или bulge. Сущность отклонена.",
            &entity
        );
        ++malformed;
        return false;
    }

    const bool closed = (groupInt(entity, 70) & 1) != 0;

    if (vertices.size() < 2) {
        diagnostic(
            diagnostics,
            DxfSeverity::Error,
            "DXF/LWPOLYLINE",
            "LWPOLYLINE содержит меньше двух валидных вершин.",
            &entity
        );
        ++malformed;
        return false;
    }

    Polygon polygon = polylineGeometry(vertices, closed, tolerance);
    if (polygon.size() < 3) {
        diagnostic(
            diagnostics,
            DxfSeverity::Error,
            "DXF/LWPOLYLINE",
            "После обработки сегментов LWPOLYLINE не образовала валидную геометрию.",
            &entity
        );
        ++malformed;
        return false;
    }

    const std::string source =
        "LWPOLYLINE#" + std::to_string(entity.index);

    if (closed) {
        loops.push_back({
            std::move(polygon),
            source,
            groupString(entity, 8, "0"),
            0.0,
            {}
        });
        loops.back().area = signedArea(loops.back().polygon);
    } else {
        parseEntityAsSegment(
            entity,
            std::move(polygon),
            segments,
            source
        );
    }

    ++supported;
    return true;
}

bool parseEntityPolyline(
    const Entity& entity,
    std::vector<Segment>& segments,
    std::vector<Loop>& loops,
    std::vector<DxfDiagnostic>& diagnostics,
    std::size_t& supported,
    std::size_t& malformed,
    double tolerance
) {
    const int flags = groupInt(entity, 70, 0);
    const int unsupportedFlags = flags & (8 | 16 | 32 | 64);
    if (unsupportedFlags != 0) {
        diagnostic(
            diagnostics,
            DxfSeverity::Warning,
            "DXF/POLYLINE",
            "3D/mesh POLYLINE flags не поддерживаются; сущность пропущена.",
            &entity
        );
        ++malformed;
        return false;
    }

    const bool closed = (flags & 1) != 0;
    const auto vertices = readPolylineVertices(
        entity,
        entity.children,
        diagnostics,
        tolerance
    );

    if (vertices.size() < 2) {
        ++malformed;
        return false;
    }

    Polygon polygon = polylineGeometry(vertices, closed, tolerance);
    if (polygon.size() < 3) {
        diagnostic(
            diagnostics,
            DxfSeverity::Error,
            "DXF/POLYLINE",
            "POLYLINE после обработки bulge не образовала валидный контур.",
            &entity
        );
        ++malformed;
        return false;
    }

    const std::string source =
        "POLYLINE#" + std::to_string(entity.index);

    if (closed) {
        loops.push_back({
            std::move(polygon),
            source,
            groupString(entity, 8, "0"),
            0.0,
            {}
        });
        loops.back().area = signedArea(loops.back().polygon);
    } else {
        parseEntityAsSegment(
            entity,
            std::move(polygon),
            segments,
            source
        );
    }

    ++supported;
    return true;
}

void collectEntities(
    const std::vector<Entity>& entities,
    std::vector<Segment>& segments,
    std::vector<Loop>& loops,
    std::vector<DxfDiagnostic>& diagnostics,
    DxfDocument& document,
    double tolerance
) {
    for (const auto& entity : entities) {
        if (entity.type == "LINE") {
            parseEntityLine(
                entity,
                segments,
                diagnostics,
                document.supportedEntities,
                document.malformedEntities
            );
        } else if (entity.type == "ARC") {
            parseEntityArc(
                entity,
                segments,
                diagnostics,
                document.supportedEntities,
                document.malformedEntities,
                tolerance
            );
        } else if (entity.type == "CIRCLE") {
            parseEntityCircle(
                entity,
                loops,
                diagnostics,
                document.supportedEntities,
                document.malformedEntities,
                tolerance
            );
        } else if (entity.type == "LWPOLYLINE") {
            parseEntityLwPolyline(
                entity,
                segments,
                loops,
                diagnostics,
                document.supportedEntities,
                document.malformedEntities,
                tolerance
            );
        } else if (entity.type == "POLYLINE") {
            parseEntityPolyline(
                entity,
                segments,
                loops,
                diagnostics,
                document.supportedEntities,
                document.malformedEntities,
                tolerance
            );
        } else {
            ++document.unsupportedEntities;
            diagnostic(
                diagnostics,
                DxfSeverity::Warning,
                "DXF/ENTITY",
                "Объект " + entity.type +
                    " не поддерживается и пропущен. Поддерживаются LINE, ARC, CIRCLE, LWPOLYLINE и POLYLINE.",
                &entity
            );
        }
    }
}

void clusterSegmentEndpoints(
    std::vector<Segment>& segments,
    double tolerance
) {
    EndpointClusterer clusterer(tolerance);

    for (auto& segment : segments) {
        segment.startNode = clusterer.node(segment.start, segment.layer);
        segment.endNode = clusterer.node(segment.end, segment.layer);

        segment.start = clusterer.point(segment.startNode);
        segment.end = clusterer.point(segment.endNode);

        if (!segment.points.empty()) {
            segment.points.front() = segment.start;
            segment.points.back() = segment.end;
        }
    }
}

Point incomingTangent(const Segment& segment, bool forward) {
    if (forward) return segment.endTangent;
    return {-segment.startTangent.x, -segment.startTangent.y};
}

Point outgoingTangent(const Segment& segment, bool forward) {
    if (forward) return segment.startTangent;
    return {-segment.endTangent.x, -segment.endTangent.y};
}

double turnCost(Point incoming, Point outgoing) {
    incoming = normalizeVector(incoming);
    outgoing = normalizeVector(outgoing);
    const double product = std::clamp(dot(incoming, outgoing), -1.0, 1.0);
    return std::acos(product);
}

std::string loopKey(const std::vector<TraversedEdge>& path) {
    std::vector<std::size_t> ids;
    ids.reserve(path.size());
    for (const auto& edge : path) ids.push_back(edge.segment);
    std::sort(ids.begin(), ids.end());

    std::string key;
    for (const auto id : ids) {
        key += std::to_string(id);
        key += ',';
    }
    return key;
}

Polygon polygonFromPath(
    const std::vector<TraversedEdge>& path,
    const std::vector<Segment>& segments,
    double tolerance
) {
    Polygon polygon;

    for (const auto& edge : path) {
        const auto& points = segments[edge.segment].points;
        if (edge.forward) {
            for (std::size_t i = 0; i < points.size(); ++i) {
                if (!polygon.empty() && i == 0) continue;
                appendUnique(polygon, points[i], tolerance * 0.05);
            }
        } else {
            for (std::size_t i = points.size(); i-- > 0;) {
                if (!polygon.empty() && i + 1 == points.size()) continue;
                appendUnique(polygon, points[i], tolerance * 0.05);
            }
        }
    }

    if (!polygon.empty() &&
        samePoint(polygon.front(), polygon.back(), tolerance * 0.05)) {
        polygon.pop_back();
    }

    return polygon;
}

std::vector<Loop> connectSegments(
    std::vector<Segment>& segments,
    std::vector<DxfDiagnostic>& diagnostics,
    double tolerance
) {
    clusterSegmentEndpoints(segments, tolerance);

    std::size_t nodeCount = 0;
    for (const auto& segment : segments) {
        nodeCount = std::max(
            nodeCount,
            std::max(segment.startNode, segment.endNode) + 1
        );
    }

    struct EdgeRef {
        std::size_t segment{};
        bool fromStart{true};
    };

    std::vector<std::vector<EdgeRef>> adjacency(nodeCount);
    for (std::size_t i = 0; i < segments.size(); ++i) {
        const auto& segment = segments[i];
        adjacency[segment.startNode].push_back({i, true});
        adjacency[segment.endNode].push_back({i, false});
    }

    std::vector<Loop> loops;
    std::unordered_set<std::string> acceptedLoops;
    std::vector<bool> failedStart(segments.size(), false);

    for (std::size_t startIndex = 0; startIndex < segments.size(); ++startIndex) {
        if (segments[startIndex].used || failedStart[startIndex]) continue;
        if (segments[startIndex].startNode == segments[startIndex].endNode) {
            failedStart[startIndex] = true;
            diagnostic(
                diagnostics,
                DxfSeverity::Warning,
                "DXF/CONNECT",
                "Сегмент имеет одинаковые начальный и конечный узлы и не образует отдельный контур."
            );
            continue;
        }

        const std::size_t initialNode = segments[startIndex].startNode;
        std::size_t currentNode = segments[startIndex].endNode;
        bool currentForward = true;
        std::vector<TraversedEdge> path{{startIndex, true}};
        std::unordered_set<std::size_t> localEdges{startIndex};
        Point incoming = incomingTangent(segments[startIndex], currentForward);
        bool closed = false;

        for (std::size_t guard = 0; guard <= segments.size(); ++guard) {
            if (currentNode == initialNode) {
                closed = true;
                break;
            }

            const EdgeRef* best = nullptr;
            double bestCost = std::numeric_limits<double>::infinity();

            for (const auto candidate : adjacency[currentNode]) {
                if (localEdges.count(candidate.segment) != 0) continue;

                const bool forward = candidate.fromStart;
                const double cost = turnCost(
                    incoming,
                    outgoingTangent(segments[candidate.segment], forward)
                );

                if (cost < bestCost) {
                    bestCost = cost;
                    best = &candidate;
                }
            }

            if (!best) break;

            const bool forward = best->fromStart;
            path.push_back({best->segment, forward});
            localEdges.insert(best->segment);

            currentNode = forward
                ? segments[best->segment].endNode
                : segments[best->segment].startNode;
            incoming = incomingTangent(
                segments[best->segment],
                forward
            );
        }

        if (!closed) {
            failedStart[startIndex] = true;
            diagnostic(
                diagnostics,
                DxfSeverity::Warning,
                "DXF/CONNECT",
                "Не удалось замкнуть цепочку " +
                    segments[startIndex].source +
                    ". Проверьте совпадение концов сегментов, layer и направление дуг."
            );
            continue;
        }

        const std::string key = loopKey(path);
        if (acceptedLoops.count(key) != 0) {
            for (const auto& edge : path) segments[edge.segment].used = true;
            continue;
        }

        Polygon polygon = polygonFromPath(path, segments, tolerance);
        const double area = signedArea(polygon);

        if (polygon.size() < 3 || std::abs(area) <= kAreaEps) {
            diagnostic(
                diagnostics,
                DxfSeverity::Warning,
                "DXF/CONNECT",
                "Замкнутая цепочка " +
                    segments[startIndex].source +
                    " имеет нулевую площадь и пропущена."
            );
            continue;
        }

        if (selfIntersects(polygon, tolerance * 0.25)) {
            diagnostic(
                diagnostics,
                DxfSeverity::Error,
                "DXF/GEOMETRY",
                "Замкнутый контур " +
                    segments[startIndex].source +
                    " самопересекается и не может использоваться для nesting."
            );
            for (const auto& edge : path) segments[edge.segment].used = true;
            continue;
        }

        acceptedLoops.insert(key);
        for (const auto& edge : path) segments[edge.segment].used = true;

        loops.push_back({
            std::move(polygon),
            "CHAIN#" + std::to_string(loops.size() + 1),
            segments[startIndex].layer,
            area,
            {}
        });
    }

    return loops;
}

Point representativePoint(const Polygon& polygon) {
    if (polygon.empty()) return {};

    Point average{};
    for (const auto& point : polygon) {
        average.x += point.x;
        average.y += point.y;
    }
    average.x /= static_cast<double>(polygon.size());
    average.y /= static_cast<double>(polygon.size());
    return average;
}

Point safeProbe(const Polygon& polygon, double tolerance) {
    if (polygon.empty()) return {};

    const Point centroid = representativePoint(polygon);
    if (pointInPolygonInclusive(centroid, polygon, tolerance)) {
        return centroid;
    }

    const double area = signedArea(polygon);
    const Point a = polygon[0];
    const Point b = polygon[1 % polygon.size()];
    const Point edge{b.x - a.x, b.y - a.y};
    Point normal = normalizeVector({
        area >= 0.0 ? -edge.y : edge.y,
        area >= 0.0 ? edge.x : -edge.x
    });

    const auto minMaxX = std::minmax_element(
        polygon.begin(),
        polygon.end(),
        [](const Point& lhs, const Point& rhs) { return lhs.x < rhs.x; }
    );
    const auto minMaxY = std::minmax_element(
        polygon.begin(),
        polygon.end(),
        [](const Point& lhs, const Point& rhs) { return lhs.y < rhs.y; }
    );

    const double scale = std::max(
        1.0,
        std::max(
            minMaxX.second->x - minMaxX.first->x,
            minMaxY.second->y - minMaxY.first->y
        )
    );

    const double offset = std::max(
        tolerance * 2.0,
        std::min(1e-3 * scale, 0.25 * tolerance)
    );

    const Point mid{
        (a.x + b.x) * 0.5,
        (a.y + b.y) * 0.5
    };

    const Point candidate{
        mid.x + normal.x * offset,
        mid.y + normal.y * offset
    };

    if (pointInPolygonInclusive(candidate, polygon, tolerance)) {
        return candidate;
    }

    return centroid;
}

void normalizeOrientation(Polygon& polygon, bool outer) {
    const double area = signedArea(polygon);
    if ((outer && area < 0.0) || (!outer && area > 0.0)) {
        std::reverse(polygon.begin(), polygon.end());
    }
}

std::vector<DxfContour> classifyLoops(
    std::vector<Loop> loops,
    double tolerance
) {
    std::vector<DxfContour> result;
    if (loops.empty()) return result;

    const std::size_t n = loops.size();
    std::vector<int> depth(n, 0);

    std::vector<Point> probes;
    probes.reserve(n);
    for (const auto& loop : loops) {
        probes.push_back(safeProbe(loop.polygon, tolerance));
    }

    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            if (i == j) continue;
            if (loops[i].layer != loops[j].layer) continue;
            if (std::abs(loops[j].area) <= std::abs(loops[i].area)) continue;

            if (pointInPolygonInclusive(
                    probes[i],
                    loops[j].polygon,
                    tolerance
                )) {
                ++depth[i];
            }
        }
    }

    std::vector<int> outerContourIndex(n, -1);

    for (std::size_t i = 0; i < n; ++i) {
        if ((depth[i] & 1) != 0) continue;

        Polygon outer = std::move(loops[i].polygon);
        normalizeOrientation(outer, true);

        const int contourIndex = static_cast<int>(result.size());
        outerContourIndex[i] = contourIndex;

        result.push_back({
            std::move(outer),
            {},
            loops[i].source,
            loops[i].layer
        });
    }

    for (std::size_t i = 0; i < n; ++i) {
        if ((depth[i] & 1) == 0) continue;

        std::size_t parent = n;
        double bestArea = std::numeric_limits<double>::infinity();

        for (std::size_t j = 0; j < n; ++j) {
            if ((depth[j] & 1) != 0) continue;
            if (loops[i].layer != loops[j].layer) continue;
            if (std::abs(loops[j].area) <= std::abs(loops[i].area)) continue;
            if (!pointInPolygonInclusive(
                    probes[i],
                    loops[j].polygon,
                    tolerance
                )) {
                continue;
            }

            const double area = std::abs(loops[j].area);
            if (area < bestArea) {
                bestArea = area;
                parent = j;
            }
        }

        if (parent == n || outerContourIndex[parent] < 0) continue;

        Polygon hole = std::move(loops[i].polygon);
        normalizeOrientation(hole, false);
        result[outerContourIndex[parent]].holes.push_back(std::move(hole));
    }

    return result;
}

} // namespace

bool DxfDocument::valid() const {
    return !contours.empty();
}

bool DxfDocument::hasErrors() const {
    return std::any_of(
        diagnostics.begin(),
        diagnostics.end(),
        [](const DxfDiagnostic& diagnostic) {
            return diagnostic.severity == DxfSeverity::Error;
        }
    );
}

bool DxfDocument::hasWarnings() const {
    return std::any_of(
        diagnostics.begin(),
        diagnostics.end(),
        [](const DxfDiagnostic& diagnostic) {
            return diagnostic.severity == DxfSeverity::Warning;
        }
    );
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

    const double tolerance = std::max(0.001, std::abs(arcToleranceMm));

    auto groups = readGroups(text, document.diagnostics);
    bool foundEntitiesSection = false;

    auto entities = readEntities(
        groups,
        document.diagnostics,
        document.entitiesRead,
        foundEntitiesSection
    );

    if (!foundEntitiesSection) {
        document.diagnostics.push_back({
            DxfSeverity::Error,
            "DXF/ENTITIES",
            "В DXF отсутствует раздел ENTITIES."
        });
        return document;
    }

    if (entities.empty()) {
        document.diagnostics.push_back({
            DxfSeverity::Error,
            "DXF/ENTITIES",
            "Раздел ENTITIES найден, но не содержит ни одной сущности."
        });
        return document;
    }

    std::vector<Segment> segments;
    std::vector<Loop> loops;

    collectEntities(
        entities,
        segments,
        loops,
        document.diagnostics,
        document,
        tolerance
    );

    auto connected = connectSegments(
        segments,
        document.diagnostics,
        std::max(1e-5, std::min(0.05, tolerance * 0.05))
    );

    loops.insert(
        loops.end(),
        std::make_move_iterator(connected.begin()),
        std::make_move_iterator(connected.end())
    );

    std::vector<Loop> validLoops;
    validLoops.reserve(loops.size());

    for (auto& loop : loops) {
        loop.area = signedArea(loop.polygon);

        if (loop.polygon.size() < 3 || std::abs(loop.area) <= kAreaEps) {
            diagnostic(
                document.diagnostics,
                DxfSeverity::Warning,
                "DXF/GEOMETRY",
                "Замкнутый контур " + loop.source +
                    " имеет недостаточное число точек или нулевую площадь."
            );
            continue;
        }

        if (selfIntersects(loop.polygon, tolerance * 0.25)) {
            diagnostic(
                document.diagnostics,
                DxfSeverity::Error,
                "DXF/GEOMETRY",
                "Контур " + loop.source +
                    " самопересекается и исключён из nesting."
            );
            continue;
        }

        validLoops.push_back(std::move(loop));
    }

    document.closedLoopsFound = validLoops.size();
    document.contours = classifyLoops(
        std::move(validLoops),
        tolerance
    );

    if (document.contours.empty()) {
        document.diagnostics.push_back({
            DxfSeverity::Error,
            "DXF/GEOMETRY",
            "Не найден ни один валидный замкнутый контур после объединения сегментов. " \
            "Проверьте замыкание LINE/ARC/POLYLINE, координаты и layer."
        });
    } else {
        document.diagnostics.push_back({
            DxfSeverity::Info,
            "DXF/GEOMETRY",
            "DXF успешно преобразован: внешние контуры, отверстия и независимые детали распознаны."
        });
    }

    return document;
}

} // namespace sheetnest
