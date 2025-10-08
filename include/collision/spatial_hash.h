#pragma once

#include "core/rigid_body.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>

namespace parallax {

/**
 * @brief Spatial hash grid for efficient broad-phase collision detection
 * 
 * Instead of checking all N bodies against each other (O(N²)),
 * we partition space into a grid and only check bodies in the same
 * or neighboring cells (O(N) on average).
 * 
 * This is critical for performance with many objects.
 */
class SpatialHash {
public:
    explicit SpatialHash(float cellSize = 10.0f);
    
    // Rebuild the grid (call every frame before collision detection)
    void clear();
    void insert(RigidBody* body);
    void build(const std::vector<RigidBody*>& bodies);
    
    // Query for potential collision pairs
    std::vector<std::pair<RigidBody*, RigidBody*>> queryPairs() const;
    
    // Query for bodies near a point or region
    std::vector<RigidBody*> queryPoint(const Vec2& point) const;
    std::vector<RigidBody*> queryRegion(const Vec2& min, const Vec2& max) const;
    
    // Settings
    void setCellSize(float size) { m_cellSize = size; }
    float getCellSize() const { return m_cellSize; }
    
    // Stats for debugging
    size_t getCellCount() const { return m_grid.size(); }
    size_t getBodyCount() const { return m_bodyCount; }

private:
    float m_cellSize;
    size_t m_bodyCount;
    
    // Hash table: cell coordinate -> list of bodies
    struct CellCoord {
        int x, y;
        
        bool operator==(const CellCoord& other) const {
            return x == other.x && y == other.y;
        }
    };
    
    struct CellHash {
        size_t operator()(const CellCoord& coord) const {
            // Hash combine using prime numbers
            return std::hash<int>()(coord.x) * 73856093 ^ 
                   std::hash<int>()(coord.y) * 19349663;
        }
    };
    
    std::unordered_map<CellCoord, std::vector<RigidBody*>, CellHash> m_grid;
    
    // Helper functions
    CellCoord getCellCoord(const Vec2& position) const;
    std::vector<CellCoord> getCellCoordsForBody(RigidBody* body) const;
    void insertIntoCell(const CellCoord& coord, RigidBody* body);
};

/**
 * @brief AABB (Axis-Aligned Bounding Box) for broad-phase collision
 */
struct AABB {
    Vec2 min;
    Vec2 max;
    
    AABB() : min(Vec2::zero()), max(Vec2::zero()) {}
    AABB(const Vec2& min, const Vec2& max) : min(min), max(max) {}
    
    bool overlaps(const AABB& other) const {
        if (max.x < other.min.x || min.x > other.max.x) return false;
        if (max.y < other.min.y || min.y > other.max.y) return false;
        return true;
    }
    
    bool contains(const Vec2& point) const {
        return point.x >= min.x && point.x <= max.x &&
               point.y >= min.y && point.y <= max.y;
    }
    
    Vec2 center() const {
        return (min + max) * 0.5f;
    }
    
    Vec2 extents() const {
        return (max - min) * 0.5f;
    }
    
    float perimeter() const {
        Vec2 d = max - min;
        return 2.0f * (d.x + d.y);
    }
    
    float area() const {
        Vec2 d = max - min;
        return d.x * d.y;
    }
    
    void expand(float amount) {
        min -= Vec2(amount, amount);
        max += Vec2(amount, amount);
    }
    
    static AABB merge(const AABB& a, const AABB& b) {
        return AABB(
            Vec2(std::min(a.min.x, b.min.x), std::min(a.min.y, b.min.y)),
            Vec2(std::max(a.max.x, b.max.x), std::max(a.max.y, b.max.y))
        );
    }
    
    // Compute AABB for a rigid body
    static AABB fromBody(const RigidBody* body);
};

} // namespace parallax
