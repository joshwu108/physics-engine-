#include "collision/spatial_hash.h"
#include <cmath>
#include <algorithm>

namespace parallax {

SpatialHash::SpatialHash(float cellSize)
    : m_cellSize(cellSize)
    , m_bodyCount(0)
{
}

void SpatialHash::clear() {
    m_grid.clear();
    m_bodyCount = 0;
}

SpatialHash::CellCoord SpatialHash::getCellCoord(const Vec2& position) const {
    return CellCoord{
        static_cast<int>(std::floor(position.x / m_cellSize)),
        static_cast<int>(std::floor(position.y / m_cellSize))
    };
}

std::vector<SpatialHash::CellCoord> SpatialHash::getCellCoordsForBody(RigidBody* body) const {
    std::vector<CellCoord> cells;
    
    // Get AABB for the body
    AABB aabb = AABB::fromBody(body);
    
    // Find all cells the AABB overlaps
    CellCoord minCell = getCellCoord(aabb.min);
    CellCoord maxCell = getCellCoord(aabb.max);
    
    for (int x = minCell.x; x <= maxCell.x; ++x) {
        for (int y = minCell.y; y <= maxCell.y; ++y) {
            cells.push_back(CellCoord{x, y});
        }
    }
    
    return cells;
}

void SpatialHash::insertIntoCell(const CellCoord& coord, RigidBody* body) {
    m_grid[coord].push_back(body);
}

void SpatialHash::insert(RigidBody* body) {
    auto cells = getCellCoordsForBody(body);
    for (const auto& cell : cells) {
        insertIntoCell(cell, body);
    }
    m_bodyCount++;
}

void SpatialHash::build(const std::vector<RigidBody*>& bodies) {
    clear();
    for (auto* body : bodies) {
        if (body && body->isAwake()) {
            insert(body);
        }
    }
}

std::vector<std::pair<RigidBody*, RigidBody*>> SpatialHash::queryPairs() const {
    std::vector<std::pair<RigidBody*, RigidBody*>> pairs;
    std::unordered_set<size_t> checked;
    
    // For each cell, check all pairs within that cell
    for (const auto& [coord, bodies] : m_grid) {
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                RigidBody* a = bodies[i];
                RigidBody* b = bodies[j];
                
                // Create unique pair ID
                uint32_t id1 = std::min(a->getId(), b->getId());
                uint32_t id2 = std::max(a->getId(), b->getId());
                size_t pairId = (static_cast<size_t>(id1) << 32) | id2;
                
                // Skip if already checked
                if (checked.count(pairId)) continue;
                checked.insert(pairId);
                
                // Skip if both are static or sleeping
                if ((a->isStatic() && b->isStatic()) ||
                    (a->isSleeping() && b->isSleeping())) {
                    continue;
                }
                
                // Check AABB overlap
                AABB aabbA = AABB::fromBody(a);
                AABB aabbB = AABB::fromBody(b);
                
                if (aabbA.overlaps(aabbB)) {
                    pairs.push_back({a, b});
                }
            }
        }
    }
    
    return pairs;
}

std::vector<RigidBody*> SpatialHash::queryPoint(const Vec2& point) const {
    CellCoord coord = getCellCoord(point);
    
    auto it = m_grid.find(coord);
    if (it != m_grid.end()) {
        return it->second;
    }
    
    return {};
}

std::vector<RigidBody*> SpatialHash::queryRegion(const Vec2& min, const Vec2& max) const {
    std::unordered_set<RigidBody*> uniqueBodies;
    
    CellCoord minCell = getCellCoord(min);
    CellCoord maxCell = getCellCoord(max);
    
    for (int x = minCell.x; x <= maxCell.x; ++x) {
        for (int y = minCell.y; y <= maxCell.y; ++y) {
            CellCoord coord{x, y};
            auto it = m_grid.find(coord);
            if (it != m_grid.end()) {
                for (auto* body : it->second) {
                    uniqueBodies.insert(body);
                }
            }
        }
    }
    
    return std::vector<RigidBody*>(uniqueBodies.begin(), uniqueBodies.end());
}

// AABB implementation

AABB AABB::fromBody(const RigidBody* body) {
    if (!body || !body->getShape()) {
        return AABB();
    }
    
    auto shape = body->getShape();
    Vec2 pos = body->getPosition();
    Mat2 rot = body->getRotationMatrix();
    
    if (shape->getType() == Shape::Type::Circle) {
        auto* circle = static_cast<const CircleShape*>(shape.get());
        float r = circle->getRadius();
        return AABB(pos - Vec2(r, r), pos + Vec2(r, r));
    }
    else if (shape->getType() == Shape::Type::Polygon || 
             shape->getType() == Shape::Type::Box) {
        auto* poly = static_cast<const PolygonShape*>(shape.get());
        const auto& vertices = poly->getVertices();
        
        if (vertices.empty()) {
            return AABB();
        }
        
        // Transform all vertices and find min/max
        Vec2 minVec = pos + rot * vertices[0];
        Vec2 maxVec = minVec;
        
        for (size_t i = 1; i < vertices.size(); ++i) {
            Vec2 v = pos + rot * vertices[i];
            minVec.x = std::min(minVec.x, v.x);
            minVec.y = std::min(minVec.y, v.y);
            maxVec.x = std::max(maxVec.x, v.x);
            maxVec.y = std::max(maxVec.y, v.y);
        }
        
        return AABB(minVec, maxVec);
    }
    
    return AABB();
}

}
