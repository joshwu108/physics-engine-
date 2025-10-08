#pragma once

#include "math/vec2.h"
#include "math/mat2.h"
#include <memory>
#include <vector>

namespace parallax {

// Forward declarations
class Shape;

/**
 * @brief Rigid body with mass properties and dynamics
 * 
 * Features:
 * - Semi-implicit Euler integration (stable for physics)
 * - Automatic sleeping for inactive bodies
 * - Support for static, kinematic, and dynamic bodies
 * - Proper mass distribution (center of mass, moment of inertia)
 */
class RigidBody {
public:
    enum class Type {
        Static,     // Infinite mass, doesn't move
        Kinematic,  // Infinite mass, velocity-controlled
        Dynamic     // Finite mass, force-driven
    };

    // Construction
    RigidBody(Type type = Type::Dynamic);
    
    // Mass properties
    void setMass(float mass);
    void setInertia(float inertia);
    void setMassAndInertia(float mass, float inertia);
    void setDensity(float density); // Auto-calculates mass from shape
    
    float getMass() const { return m_mass; }
    float getInvMass() const { return m_invMass; }
    float getInertia() const { return m_inertia; }
    float getInvInertia() const { return m_invInertia; }
    
    // Transform
    Vec2 getPosition() const { return m_position; }
    float getAngle() const { return m_angle; }
    Mat2 getRotationMatrix() const { return Mat2::rotation(m_angle); }
    
    void setPosition(const Vec2& pos) { m_position = pos; }
    void setAngle(float angle) { m_angle = angle; }
    void setTransform(const Vec2& pos, float angle) {
        m_position = pos;
        m_angle = angle;
    }
    
    // Velocity
    Vec2 getLinearVelocity() const { return m_linearVelocity; }
    float getAngularVelocity() const { return m_angularVelocity; }
    
    void setLinearVelocity(const Vec2& vel) { 
        m_linearVelocity = vel; 
        if (m_type == Type::Dynamic) {
            wakeUp();
        }
    }
    
    void setAngularVelocity(float vel) { 
        m_angularVelocity = vel; 
        if (m_type == Type::Dynamic) {
            wakeUp();
        }
    }
    
    // Get velocity at a world point
    Vec2 getVelocityAtPoint(const Vec2& worldPoint) const {
        Vec2 r = worldPoint - m_position;
        return m_linearVelocity + r.perpendicular() * m_angularVelocity;
    }
    
    // Forces and impulses
    void applyForce(const Vec2& force);
    void applyForceAtPoint(const Vec2& force, const Vec2& worldPoint);
    void applyImpulse(const Vec2& impulse);
    void applyImpulseAtPoint(const Vec2& impulse, const Vec2& worldPoint);
    void applyAngularImpulse(float impulse);
    
    // Material properties
    void setRestitution(float restitution) { m_restitution = restitution; }
    void setFriction(float friction) { m_friction = friction; }
    void setLinearDamping(float damping) { m_linearDamping = damping; }
    void setAngularDamping(float damping) { m_angularDamping = damping; }
    
    float getRestitution() const { return m_restitution; }
    float getFriction() const { return m_friction; }
    float getLinearDamping() const { return m_linearDamping; }
    float getAngularDamping() const { return m_angularDamping; }
    
    // Type
    Type getType() const { return m_type; }
    void setType(Type type);
    bool isStatic() const { return m_type == Type::Static; }
    bool isKinematic() const { return m_type == Type::Kinematic; }
    bool isDynamic() const { return m_type == Type::Dynamic; }
    
    // Sleeping (for performance)
    bool isAwake() const { return m_isAwake; }
    bool isSleeping() const { return !m_isAwake; }
    void wakeUp();
    void putToSleep();
    void setSleepingAllowed(bool allowed) { m_allowSleep = allowed; }
    
    // Shape
    void setShape(std::shared_ptr<Shape> shape) { m_shape = shape; }
    std::shared_ptr<Shape> getShape() const { return m_shape; }
    
    // Integration (called by physics world)
    void integrate(float dt);
    void clearForces();
    
    // Island solver support
    int getIslandIndex() const { return m_islandIndex; }
    void setIslandIndex(int index) { m_islandIndex = index; }
    
    // User data
    void setUserData(void* data) { m_userData = data; }
    void* getUserData() const { return m_userData; }
    
    // Unique ID
    uint32_t getId() const { return m_id; }

private:
    static uint32_t s_nextId;
    uint32_t m_id;
    
    // Type
    Type m_type;
    
    // Transform
    Vec2 m_position;
    float m_angle;
    
    // Velocity
    Vec2 m_linearVelocity;
    float m_angularVelocity;
    
    // Forces (accumulated)
    Vec2 m_force;
    float m_torque;
    
    // Mass properties
    float m_mass;
    float m_invMass;
    float m_inertia;
    float m_invInertia;
    
    // Material properties
    float m_restitution;  // Bounciness (0-1)
    float m_friction;     // Surface friction (0-1)
    float m_linearDamping;  // Air resistance
    float m_angularDamping; // Rotational resistance
    
    // Sleeping
    bool m_isAwake;
    bool m_allowSleep;
    float m_sleepTimer;
    static constexpr float SLEEP_TIME_THRESHOLD = 0.5f;
    static constexpr float SLEEP_LINEAR_VELOCITY_SQ = 0.01f;
    static constexpr float SLEEP_ANGULAR_VELOCITY_SQ = 0.01f;
    
    // Shape
    std::shared_ptr<Shape> m_shape;
    
    // Island solver
    int m_islandIndex;
    
    // User data
    void* m_userData;
    
    void updateMassData();
    void updateSleeping(float dt);
};

/**
 * @brief Base class for collision shapes
 */
class Shape {
public:
    enum class Type {
        Circle,
        Polygon,
        Box
    };
    
    virtual ~Shape() = default;
    virtual Type getType() const = 0;
    virtual float computeArea() const = 0;
    virtual float computeInertia(float mass) const = 0;
    virtual Vec2 computeCentroid() const = 0;
};

/**
 * @brief Circle shape
 */
class CircleShape : public Shape {
public:
    explicit CircleShape(float radius) : m_radius(radius) {}
    
    Type getType() const override { return Type::Circle; }
    float getRadius() const { return m_radius; }
    
    float computeArea() const override;
    float computeInertia(float mass) const override;
    Vec2 computeCentroid() const override { return Vec2::zero(); }

private:
    float m_radius;
};

/**
 * @brief Polygon shape (convex only)
 */
class PolygonShape : public Shape {
public:
    PolygonShape() = default;
    explicit PolygonShape(const std::vector<Vec2>& vertices);
    
    Type getType() const override { return Type::Polygon; }
    
    void setVertices(const std::vector<Vec2>& vertices);
    const std::vector<Vec2>& getVertices() const { return m_vertices; }
    const std::vector<Vec2>& getNormals() const { return m_normals; }
    
    float computeArea() const override;
    float computeInertia(float mass) const override;
    Vec2 computeCentroid() const override;
    
    // Helper to create a box
    static PolygonShape makeBox(float width, float height);

private:
    std::vector<Vec2> m_vertices;
    std::vector<Vec2> m_normals;
    
    void computeNormals();
    bool validate() const;
};

} // namespace parallax
