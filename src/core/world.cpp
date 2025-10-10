#include "core/world.h"
#include <algorithm>

namespace parallax {

World::World()
    : m_gravity(0.0f, -9.8f)
    , m_spatialHash(10.0f)
{}

World::World(const Vec2& gravity)
    : m_gravity(gravity)
    , m_spatialHash(10.0f)
{}

void World::step(float dt) {
    applyForces(dt);
    detectCollisions();
    solveConstraints(dt);
    integrate(dt);
}

void World::applyForces(float dt) {
    for(auto& body: m_bodies) {
        if (!body || !body->isDynamic()) continue;
        Vec2 gravityForce = m_gravity * body->getMass();
        body->applyForce(gravityForce * dt);
    }
}

void World::detectCollisions() {
    m_contacts.clear();
    m_spatialHash.build(m_bodies);
    auto pairs = m_spatialHash.queryPairs();
    for(const auto& [bodyA, bodyB] : pairs) {
        Manifold manifold;
        if(CollisionDetector::detect(bodyA, bodyB, manifold)) {
            m_contacts.push_back(manifold);
        }
    }
}

void World::solveConstraints(float dt) {
    m_solver.setContacts(m_contacts);
    m_solver.solve(dt);
}

void World::integrate(float dt) {
    for(auto* body : m_bodies) {
        if (!body) continue;
        body->integrate(dt);
    }
}

void World::addBody(RigidBody* body) {
    if(!body) return;
    auto it = std::find(m_bodies.begin(), m_bodies.end(), body);
    if(it == m_bodies.end()) {
        m_bodies.push_back(body);
    }
}

void World::removeBody(RigidBody* body) {
    auto it = std:: find(m_bodies.begin(), m_bodies.end(), body);
    if (it != m_bodies.end()) {
        m_bodies.erase(it);
    }
}

void World::clearBodies() {
    m_bodies.clear();
}

void World::addConstraint(std::shared_ptr<Constraint> constraint) {
    m_solver.addConstraint(constraint);
}

void World::removeConstraint(std::shared_ptr<Constraint> constraint) {
    m_solver.removeConstraint(constraint);
}

void World::clearConstraints() {
    m_solver.clearConstraints();
}

void World::setSpatialHashCellSize(float size) {
    m_spatialHash.setCellSize(size);
}

RigidBody* World::queryPoint(const Vec2& point) {
    auto bodies = m_spatialHash.queryPoint(point);

    for(auto* body : bodies) {
        if(!body || !body->getShape()) continue;

        Vec2 localPoint = body->getRotationMatrix().transpose() * (point - body->getPosition());
        auto shape = body->getShape();

        if(shape->getType() == Shape::Type::Circle) {
            auto* circle = static_cast<const CircleShape*>(shape.get());
            if (localPoint.lengthSquared() <= circle->getRadius() * circle->getRadius()) {
                return body;
            }
        } else if (shape->getType() == Shape::Type::Polygon || shape->getType() == Shape::Type::Box) {
            auto* poly = static_cast<const PolygonShape*>(shape.get());
            const auto& vertices = poly->getVertices();
            const auto& normals = poly->getNormals();

            bool inside = true;
            for(size_t i = 0; i < vertices.size(); ++i) {
                Vec2 toPoint = localPoint - vertices[i];
                if (toPoint.dot(normals[i]) > 0.0f) {
                    inside = false;
                    break;
                }
            }
            if (inside) {
                return body;
            }
        }
    }
    return nullptr;
}

std::vector<RigidBody*> World::queryRegion(const Vec2& min, const Vec2& max) {
    return m_spatialHash.queryRegion(min, max);
}
} // namespace parallax
