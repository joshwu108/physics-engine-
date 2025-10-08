#pragma once

#include "constraints/constraint.h"
#include "math/vec2.h"

namespace parallax {

/**
 * @brief Constraint to mimic a spring between two bodies using Hooke's law
 */
class SpringConstraint : public Constraint {
public:
    SpringConstraint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec2& anchorA,
        const Vec2& anchorB,
        float restLength,
        float stiffness,
        float damping = 0.1f
    );

    void prepare(float dt) override;
    void solve() override;

    // Getters
    Vec2 getAnchorA() const {
        return m_localAnchorA;
    }

    Vec2 getAnchorB() const {
        return m_localAnchorB;
    }

    float getRestLength() const {
        return m_restLength;
    }

    float getStiffness() const {
        return m_stiffness;
    }

    float getDamping() const {
        return m_damping;
    }

    // Setters
    void setRestLength(float newLength) {
        m_restLength = newLength;
    }

    void setStiffness(float newStiffness) {
        m_stiffness = newStiffness;
    }

    void setDamping(float newDamping) {
        m_damping = newDamping;
    }

private:
    Vec2 m_localAnchorA;
    Vec2 m_localAnchorB;
    float m_restLength;
    float m_stiffness;
    float m_damping;

    Vec2 m_rA;
    Vec2 m_rB;
    Vec2 m_normal;
    float m_mass;
};

} // namespace parallax