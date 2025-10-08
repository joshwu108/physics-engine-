#pragma once

#include "core/rigid_body.h"
#include "math/vec2.h"
#include <vector>

namespace parallax {

/**
 * @brief Contact point information for collision resolution
 */
struct Contact {
    RigidBody* bodyA;
    RigidBody* bodyB;
    
    Vec2 position;      // Contact position in world space
    Vec2 normal;        // Normal from A to B
    float penetration;  // How deep they're overlapping
    
    // Contact properties (derived from bodies)
    float restitution;  // Bounciness
    float friction;     // Surface friction
    
    Contact()
        : bodyA(nullptr)
        , bodyB(nullptr)
        , position(Vec2::zero())
        , normal(Vec2::zero())
        , penetration(0.0f)
        , restitution(0.0f)
        , friction(0.0f)
    {}
};

/**
 * @brief Manifold containing multiple contact points
 */
struct Manifold {
    RigidBody* bodyA;
    RigidBody* bodyB;
    
    std::vector<Contact> contacts;
    Vec2 normal;        // Normal from A to B
    float penetration;  // Maximum penetration
    
    Manifold() 
        : bodyA(nullptr)
        , bodyB(nullptr)
        , normal(Vec2::zero())
        , penetration(0.0f)
    {}
    
    bool hasContacts() const { return !contacts.empty(); }
    size_t getContactCount() const { return contacts.size(); }
};

/**
 * @brief Narrow-phase collision detector using SAT
 * 
 * Implements the Separating Axis Theorem for accurate collision detection:
 * - Circle vs Circle
 * - Circle vs Polygon
 * - Polygon vs Polygon
 * 
 * Generates contact manifolds with penetration depth and contact points.
 */
class CollisionDetector {
public:
    CollisionDetector() = default;
    
    // Detect collision and generate manifold
    static bool detect(RigidBody* a, RigidBody* b, Manifold& manifold);
    
private:
    // Shape-specific detection methods
    static bool detectCircleCircle(
        RigidBody* a, RigidBody* b,
        const CircleShape* circleA, const CircleShape* circleB,
        Manifold& manifold
    );
    
    static bool detectCirclePolygon(
        RigidBody* a, RigidBody* b,
        const CircleShape* circle, const PolygonShape* polygon,
        Manifold& manifold
    );
    
    static bool detectPolygonPolygon(
        RigidBody* a, RigidBody* b,
        const PolygonShape* polyA, const PolygonShape* polyB,
        Manifold& manifold
    );
    
    // SAT helper: find minimum separating axis
    struct SATResult {
        bool separated;
        float penetration;
        Vec2 normal;
        int faceIndex;
    };
    
    static SATResult findMinSeparation(
        const std::vector<Vec2>& verticesA,
        const std::vector<Vec2>& normalsA,
        const std::vector<Vec2>& verticesB
    );
    
    // Clipping for contact manifold generation
    static void clipPolygonEdge(
        const std::vector<Vec2>& polygon,
        const Vec2& planeNormal,
        float planeOffset,
        std::vector<Vec2>& output
    );
    
    static std::vector<Vec2> sutherland_hodgman(
        const std::vector<Vec2>& subject,
        const std::vector<Vec2>& clip
    );
    
    // Transform vertices from local to world space
    static std::vector<Vec2> transformVertices(
        const std::vector<Vec2>& local,
        const Vec2& position,
        const Mat2& rotation
    );
    
    // Find the support point (furthest point in a direction)
    static int findSupportPoint(
        const std::vector<Vec2>& vertices,
        const Vec2& direction
    );
    
    // Find the closest point on a line segment to a point
    static Vec2 closestPointOnSegment(
        const Vec2& point,
        const Vec2& a,
        const Vec2& b
    );
};

} // namespace parallax
