#include "core/rigid_body.h"
#include <cmath>
#include <algorithm>
#include <cassert>

namespace parallax {

uint32_t RigidBody::s_nextId = 1;

RigidBody::RigidBody(Type type)
    : m_id(s_nextId++)
    , m_type(type)
    , m_position(Vec2::zero())
    , m_angle(0.0f)
    , m_linearVelocity(Vec2::zero())
    , m_angularVelocity(0.0f)
    , m_force(Vec2::zero())
    , m_torque(0.0f)
    , m_mass(1.0f)
    , m_invMass(1.0f)
    , m_inertia(1.0f)
    , m_invInertia(1.0f)
    , m_restitution(0.5f)
    , m_friction(0.3f)
    , m_linearDamping(0.01f)
    , m_angularDamping(0.01f)
    , m_isAwake(true)
    , m_allowSleep(true)
    , m_sleepTimer(0.0f)
    , m_shape(nullptr)
    , m_islandIndex(-1)
    , m_userData(nullptr)
{
    if (type != Type::Dynamic) {
        m_invMass = 0.0f;
        m_invInertia = 0.0f;
    }
}

void RigidBody::setMass(float mass) {
    if (m_type != Type::Dynamic) return;
    
    m_mass = mass;
    if (mass > 0.0f) {
        m_invMass = 1.0f / mass;
    } else {
        m_invMass = 0.0f;
    }
}

void RigidBody::setInertia(float inertia) {
    if (m_type != Type::Dynamic) return;
    
    m_inertia = inertia;
    if (inertia > 0.0f) {
        m_invInertia = 1.0f / inertia;
    } else {
        m_invInertia = 0.0f;
    }
}

void RigidBody::setMassAndInertia(float mass, float inertia) {
    setMass(mass);
    setInertia(inertia);
}

void RigidBody::setDensity(float density) {
    if (!m_shape || m_type != Type::Dynamic) return;
    
    float area = m_shape->computeArea();
    float mass = density * area;
    setMass(mass);
    
    // Inertia relative to center of mass
    float inertia = m_shape->computeInertia(mass);
    setInertia(inertia);
}

void RigidBody::setType(Type type) {
    if (m_type == type) return;
    
    m_type = type;
    
    if (type == Type::Dynamic) {
        m_invMass = (m_mass > 0.0f) ? (1.0f / m_mass) : 0.0f;
        m_invInertia = (m_inertia > 0.0f) ? (1.0f / m_inertia) : 0.0f;
    } else {
        m_invMass = 0.0f;
        m_invInertia = 0.0f;
        m_linearVelocity = Vec2::zero();
        m_angularVelocity = 0.0f;
    }
    
    wakeUp();
}

void RigidBody::applyForce(const Vec2& force) {
    if (m_type != Type::Dynamic) return;
    m_force += force;
    wakeUp();
}

void RigidBody::applyForceAtPoint(const Vec2& force, const Vec2& worldPoint) {
    if (m_type != Type::Dynamic) return;
    
    m_force += force;
    Vec2 r = worldPoint - m_position;
    m_torque += r.cross(force);
    wakeUp();
}

void RigidBody::applyImpulse(const Vec2& impulse) {
    if (m_type != Type::Dynamic) return;
    
    m_linearVelocity += impulse * m_invMass;
    wakeUp();
}

void RigidBody::applyImpulseAtPoint(const Vec2& impulse, const Vec2& worldPoint) {
    if (m_type != Type::Dynamic) return;
    
    m_linearVelocity += impulse * m_invMass;
    Vec2 r = worldPoint - m_position;
    m_angularVelocity += r.cross(impulse) * m_invInertia;
    wakeUp();
}

void RigidBody::applyAngularImpulse(float impulse) {
    if (m_type != Type::Dynamic) return;
    
    m_angularVelocity += impulse * m_invInertia;
    wakeUp();
}

void RigidBody::wakeUp() {
    if (m_type != Type::Dynamic) return;
    
    m_isAwake = true;
    m_sleepTimer = 0.0f;
}

void RigidBody::putToSleep() {
    if (!m_allowSleep || m_type != Type::Dynamic) return;
    
    m_isAwake = false;
    m_linearVelocity = Vec2::zero();
    m_angularVelocity = 0.0f;
    m_force = Vec2::zero();
    m_torque = 0.0f;
}

void RigidBody::integrate(float dt) {
    if (m_type != Type::Dynamic || !m_isAwake) return;
    
    // Semi-implicit Euler integration (more stable than explicit Euler)
    
    // Update velocity from forces
    Vec2 acceleration = m_force * m_invMass;
    m_linearVelocity += acceleration * dt;
    
    float angularAcceleration = m_torque * m_invInertia;
    m_angularVelocity += angularAcceleration * dt;
    
    // Apply damping
    m_linearVelocity *= 1.0f / (1.0f + dt * m_linearDamping);
    m_angularVelocity *= 1.0f / (1.0f + dt * m_angularDamping);
    
    // Update position from velocity
    m_position += m_linearVelocity * dt;
    m_angle += m_angularVelocity * dt;
    
    // Update sleeping
    updateSleeping(dt);
    
    // Clear forces for next frame
    clearForces();
}

void RigidBody::clearForces() {
    m_force = Vec2::zero();
    m_torque = 0.0f;
}

void RigidBody::updateSleeping(float dt) {
    if (!m_allowSleep || m_type != Type::Dynamic) return;
    
    float linearVelSq = m_linearVelocity.lengthSquared();
    float angularVelSq = m_angularVelocity * m_angularVelocity;
    
    if (linearVelSq < SLEEP_LINEAR_VELOCITY_SQ && 
        angularVelSq < SLEEP_ANGULAR_VELOCITY_SQ) {
        m_sleepTimer += dt;
        
        if (m_sleepTimer >= SLEEP_TIME_THRESHOLD) {
            putToSleep();
        }
    } else {
        m_sleepTimer = 0.0f;
    }
}

// CircleShape implementation

float CircleShape::computeArea() const {
    return M_PI * m_radius * m_radius;
}

float CircleShape::computeInertia(float mass) const {
    // I = (1/2) * m * r^2 for a solid circle
    return 0.5f * mass * m_radius * m_radius;
}

// PolygonShape implementation

PolygonShape::PolygonShape(const std::vector<Vec2>& vertices) {
    setVertices(vertices);
}

void PolygonShape::setVertices(const std::vector<Vec2>& vertices) {
    m_vertices = vertices;
    computeNormals();
    assert(validate());
}

void PolygonShape::computeNormals() {
    m_normals.clear();
    m_normals.reserve(m_vertices.size());
    
    for (size_t i = 0; i < m_vertices.size(); ++i) {
        size_t next = (i + 1) % m_vertices.size();
        Vec2 edge = m_vertices[next] - m_vertices[i];
        Vec2 normal = edge.perpendicular().normalized();
        m_normals.push_back(normal);
    }
}

bool PolygonShape::validate() const {
    if (m_vertices.size() < 3) return false;
    
    // Check if polygon is convex (all cross products have same sign)
    float sign = 0.0f;
    for (size_t i = 0; i < m_vertices.size(); ++i) {
        size_t j = (i + 1) % m_vertices.size();
        size_t k = (i + 2) % m_vertices.size();
        
        Vec2 e1 = m_vertices[j] - m_vertices[i];
        Vec2 e2 = m_vertices[k] - m_vertices[j];
        float cross = e1.cross(e2);
        
        if (i == 0) {
            sign = cross;
        } else if (cross * sign < 0.0f) {
            return false; // Not convex
        }
    }
    
    return true;
}

float PolygonShape::computeArea() const {
    float area = 0.0f;
    
    for (size_t i = 0; i < m_vertices.size(); ++i) {
        size_t next = (i + 1) % m_vertices.size();
        area += m_vertices[i].cross(m_vertices[next]);
    }
    
    return std::abs(area) * 0.5f;
}

Vec2 PolygonShape::computeCentroid() const {
    Vec2 centroid = Vec2::zero();
    float area = 0.0f;
    
    for (size_t i = 0; i < m_vertices.size(); ++i) {
        size_t next = (i + 1) % m_vertices.size();
        
        float cross = m_vertices[i].cross(m_vertices[next]);
        area += cross;
        centroid += (m_vertices[i] + m_vertices[next]) * cross;
    }
    
    area *= 0.5f;
    if (std::abs(area) > 1e-6f) {
        centroid /= (6.0f * area);
    }
    
    return centroid;
}

float PolygonShape::computeInertia(float mass) const {
    float numerator = 0.0f;
    float denominator = 0.0f;
    
    for (size_t i = 0; i < m_vertices.size(); ++i) {
        size_t next = (i + 1) % m_vertices.size();
        
        const Vec2& v1 = m_vertices[i];
        const Vec2& v2 = m_vertices[next];
        
        float cross = std::abs(v1.cross(v2));
        numerator += cross * (v1.dot(v1) + v1.dot(v2) + v2.dot(v2));
        denominator += cross;
    }
    
    if (denominator < 1e-6f) return 0.0f;
    
    return (mass / 6.0f) * (numerator / denominator);
}

PolygonShape PolygonShape::makeBox(float width, float height) {
    float w = width * 0.5f;
    float h = height * 0.5f;
    
    std::vector<Vec2> vertices = {
        Vec2(-w, -h),
        Vec2( w, -h),
        Vec2( w,  h),
        Vec2(-w,  h)
    };
    
    return PolygonShape(vertices);
}

} // namespace parallax
