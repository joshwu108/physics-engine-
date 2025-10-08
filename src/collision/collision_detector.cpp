#include "collision/collision_detector.h"
#include <algorithm>
#include <limits>
#include <cmath>

namespace parallax {

bool CollisionDetector::detect(RigidBody* a, RigidBody* b, Manifold& manifold) {
    if (!a || !b || !a->getShape() || !b->getShape()) {
        return false;
    }
    
    manifold = Manifold();
    manifold.bodyA = a;
    manifold.bodyB = b;
    
    auto shapeA = a->getShape();
    auto shapeB = b->getShape();
    
    auto typeA = shapeA->getType();
    auto typeB = shapeB->getType();
    
    // Circle vs Circle
    if (typeA == Shape::Type::Circle && typeB == Shape::Type::Circle) {
        return detectCircleCircle(
            a, b,
            static_cast<const CircleShape*>(shapeA.get()),
            static_cast<const CircleShape*>(shapeB.get()),
            manifold
        );
    }
    
    // Circle vs Polygon
    if (typeA == Shape::Type::Circle && 
        (typeB == Shape::Type::Polygon || typeB == Shape::Type::Box)) {
        return detectCirclePolygon(
            a, b,
            static_cast<const CircleShape*>(shapeA.get()),
            static_cast<const PolygonShape*>(shapeB.get()),
            manifold
        );
    }
    
    // Polygon vs Circle (swap order)
    if ((typeA == Shape::Type::Polygon || typeA == Shape::Type::Box) && 
        typeB == Shape::Type::Circle) {
        bool result = detectCirclePolygon(
            b, a,
            static_cast<const CircleShape*>(shapeB.get()),
            static_cast<const PolygonShape*>(shapeA.get()),
            manifold
        );
        if (result) {
            // Flip normal and swap bodies
            manifold.normal = -manifold.normal;
            std::swap(manifold.bodyA, manifold.bodyB);
            for (auto& contact : manifold.contacts) {
                std::swap(contact.bodyA, contact.bodyB);
                contact.normal = -contact.normal;
            }
        }
        return result;
    }
    
    // Polygon vs Polygon
    if ((typeA == Shape::Type::Polygon || typeA == Shape::Type::Box) &&
        (typeB == Shape::Type::Polygon || typeB == Shape::Type::Box)) {
        return detectPolygonPolygon(
            a, b,
            static_cast<const PolygonShape*>(shapeA.get()),
            static_cast<const PolygonShape*>(shapeB.get()),
            manifold
        );
    }
    
    return false;
}

bool CollisionDetector::detectCircleCircle(
    RigidBody* a, RigidBody* b,
    const CircleShape* circleA, const CircleShape* circleB,
    Manifold& manifold)
{
    Vec2 posA = a->getPosition();
    Vec2 posB = b->getPosition();
    
    Vec2 delta = posB - posA;
    float distSq = delta.lengthSquared();
    float radiusSum = circleA->getRadius() + circleB->getRadius();
    
    if (distSq >= radiusSum * radiusSum) {
        return false; // No collision
    }
    
    float dist = std::sqrt(distSq);
    
    Contact contact;
    contact.bodyA = a;
    contact.bodyB = b;
    contact.penetration = radiusSum - dist;
    
    if (dist > 1e-6f) {
        contact.normal = delta / dist;
    } else {
        // Circles are at same position, use arbitrary normal
        contact.normal = Vec2(1.0f, 0.0f);
    }
    
    contact.position = posA + contact.normal * circleA->getRadius();
    contact.restitution = std::min(a->getRestitution(), b->getRestitution());
    contact.friction = std::sqrt(a->getFriction() * b->getFriction());
    
    manifold.contacts.push_back(contact);
    manifold.normal = contact.normal;
    manifold.penetration = contact.penetration;
    
    return true;
}

bool CollisionDetector::detectCirclePolygon(
    RigidBody* a, RigidBody* b,
    const CircleShape* circle, const PolygonShape* polygon,
    Manifold& manifold)
{
    Vec2 circlePos = a->getPosition();
    Vec2 polyPos = b->getPosition();
    Mat2 polyRot = b->getRotationMatrix();
    
    // Transform circle center to polygon's local space
    Vec2 localCirclePos = polyRot.transpose() * (circlePos - polyPos);
    
    // Find closest point on polygon to circle center
    const auto& vertices = polygon->getVertices();
    float minDistSq = std::numeric_limits<float>::max();
    Vec2 closestPoint;
    Vec2 closestNormal;
    
    for (size_t i = 0; i < vertices.size(); ++i) {
        size_t next = (i + 1) % vertices.size();
        
        Vec2 closest = closestPointOnSegment(localCirclePos, vertices[i], vertices[next]);
        float distSq = (closest - localCirclePos).lengthSquared();
        
        if (distSq < minDistSq) {
            minDistSq = distSq;
            closestPoint = closest;
            closestNormal = polygon->getNormals()[i];
        }
    }
    
    float radius = circle->getRadius();
    if (minDistSq >= radius * radius) {
        return false; // No collision
    }
    
    float dist = std::sqrt(minDistSq);
    
    Contact contact;
    contact.bodyA = a;
    contact.bodyB = b;
    contact.penetration = radius - dist;
    
    if (dist > 1e-6f) {
        Vec2 localNormal = (localCirclePos - closestPoint) / dist;
        contact.normal = polyRot * localNormal;
    } else {
        // Circle center is on polygon edge
        contact.normal = polyRot * closestNormal;
    }
    
    contact.position = polyPos + polyRot * closestPoint;
    contact.restitution = std::min(a->getRestitution(), b->getRestitution());
    contact.friction = std::sqrt(a->getFriction() * b->getFriction());
    
    manifold.contacts.push_back(contact);
    manifold.normal = contact.normal;
    manifold.penetration = contact.penetration;
    
    return true;
}

bool CollisionDetector::detectPolygonPolygon(
    RigidBody* a, RigidBody* b,
    const PolygonShape* polyA, const PolygonShape* polyB,
    Manifold& manifold)
{
    // Transform vertices to world space
    auto vertsA = transformVertices(polyA->getVertices(), a->getPosition(), a->getRotationMatrix());
    auto vertsB = transformVertices(polyB->getVertices(), b->getPosition(), b->getRotationMatrix());
    
    // Transform normals to world space
    Mat2 rotA = a->getRotationMatrix();
    Mat2 rotB = b->getRotationMatrix();
    
    std::vector<Vec2> normalsA;
    for (const auto& n : polyA->getNormals()) {
        normalsA.push_back(rotA * n);
    }
    
    std::vector<Vec2> normalsB;
    for (const auto& n : polyB->getNormals()) {
        normalsB.push_back(rotB * n);
    }
    
    // SAT: Check separation along A's normals
    auto resultA = findMinSeparation(vertsA, normalsA, vertsB);
    if (resultA.separated) {
        return false;
    }
    
    // SAT: Check separation along B's normals
    auto resultB = findMinSeparation(vertsB, normalsB, vertsA);
    if (resultB.separated) {
        return false;
    }
    
    // Choose the axis with minimum penetration
    bool useA = resultA.penetration < resultB.penetration;
    
    if (useA) {
        manifold.normal = resultA.normal;
        manifold.penetration = resultA.penetration;
        
        // Generate contact points
        Contact contact;
        contact.bodyA = a;
        contact.bodyB = b;
        contact.normal = resultA.normal;
        contact.penetration = resultA.penetration;
        contact.position = (a->getPosition() + b->getPosition()) * 0.5f;
        contact.restitution = std::min(a->getRestitution(), b->getRestitution());
        contact.friction = std::sqrt(a->getFriction() * b->getFriction());
        
        manifold.contacts.push_back(contact);
    } else {
        manifold.normal = -resultB.normal;
        manifold.penetration = resultB.penetration;
        
        Contact contact;
        contact.bodyA = a;
        contact.bodyB = b;
        contact.normal = -resultB.normal;
        contact.penetration = resultB.penetration;
        contact.position = (a->getPosition() + b->getPosition()) * 0.5f;
        contact.restitution = std::min(a->getRestitution(), b->getRestitution());
        contact.friction = std::sqrt(a->getFriction() * b->getFriction());
        
        manifold.contacts.push_back(contact);
    }
    
    return true;
}

CollisionDetector::SATResult CollisionDetector::findMinSeparation(
    const std::vector<Vec2>& verticesA,
    const std::vector<Vec2>& normalsA,
    const std::vector<Vec2>& verticesB)
{
    SATResult result;
    result.separated = false;
    result.penetration = std::numeric_limits<float>::max();
    result.faceIndex = -1;
    
    for (size_t i = 0; i < normalsA.size(); ++i) {
        const Vec2& normal = normalsA[i];
        
        // Project A onto axis
        float minA = std::numeric_limits<float>::max();
        float maxA = std::numeric_limits<float>::lowest();
        for (const auto& v : verticesA) {
            float proj = v.dot(normal);
            minA = std::min(minA, proj);
            maxA = std::max(maxA, proj);
        }
        
        // Project B onto axis
        float minB = std::numeric_limits<float>::max();
        float maxB = std::numeric_limits<float>::lowest();
        for (const auto& v : verticesB) {
            float proj = v.dot(normal);
            minB = std::min(minB, proj);
            maxB = std::max(maxB, proj);
        }
        
        // Check for separation
        if (maxA < minB || maxB < minA) {
            result.separated = true;
            return result;
        }
        
        // Calculate penetration
        float penetration = std::min(maxA - minB, maxB - minA);
        
        if (penetration < result.penetration) {
            result.penetration = penetration;
            result.normal = normal;
            result.faceIndex = static_cast<int>(i);
        }
    }
    
    return result;
}

std::vector<Vec2> CollisionDetector::transformVertices(
    const std::vector<Vec2>& local,
    const Vec2& position,
    const Mat2& rotation)
{
    std::vector<Vec2> world;
    world.reserve(local.size());
    
    for (const auto& v : local) {
        world.push_back(position + rotation * v);
    }
    
    return world;
}

int CollisionDetector::findSupportPoint(
    const std::vector<Vec2>& vertices,
    const Vec2& direction)
{
    int bestIndex = 0;
    float maxDot = vertices[0].dot(direction);
    
    for (size_t i = 1; i < vertices.size(); ++i) {
        float d = vertices[i].dot(direction);
        if (d > maxDot) {
            maxDot = d;
            bestIndex = static_cast<int>(i);
        }
    }
    
    return bestIndex;
}

Vec2 CollisionDetector::closestPointOnSegment(
    const Vec2& point,
    const Vec2& a,
    const Vec2& b)
{
    Vec2 ab = b - a;
    Vec2 ap = point - a;
    
    float t = ap.dot(ab) / ab.dot(ab);
    t = std::clamp(t, 0.0f, 1.0f);
    
    return a + ab * t;
}

} // namespace parallax
