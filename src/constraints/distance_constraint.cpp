#include "constraints/distance_constraint.h"
#include <cmath>

namespace parallax {
    DistanceConstraint::DistanceConstraint(
        RigidBody* bodyA, 
        RigidBody* bodyB, 
        const Vec2& anchorA, 
        const Vec2& anchorB, 
        float distance)
        : Constraint(bodyA, bodyB)
        , m_localAnchorA(anchorA)
        , m_localAnchorB(anchorB)
        , m_distance(distance)
        , m_impulseSum(0.0f)
    {}
    
    void DistanceConstraint::prepare(float dt) {
        if (!m_enabled) return;

        // Fetch rotation matrices and transform anchors to world space
        Mat2 rotA = m_bodyA->getRotationMatrix();
        Mat2 rotB = m_bodyB->getRotationMatrix();
        m_rA = rotA * m_localAnchorA;
        m_rB = rotB * m_localAnchorB;

        // World positions of anchors
        Vec2 pA = m_bodyA->getPosition() + m_rA;
        Vec2 pB = m_bodyB->getPosition() + m_rB;

        Vec2 delta = pB - pA;
        float currentDist = delta.length();

        if (currentDist < 1e-6f) {
            m_normal = Vec2(1.0f, 0.0f);
        } else {
            m_normal = delta / currentDist;
        }

        // Find effective mass
        float rnA = m_rA.cross(m_normal);
        float rnB = m_rB.cross(m_normal);
        float massSum = m_bodyA->getInvMass() + m_bodyB->getInvMass();
        float inertiaSum = rnA * rnA * m_bodyA->getInvInertia() + 
                          rnB * rnB * m_bodyB->getInvInertia();
        m_mass = massSum + inertiaSum;
        if (m_mass > 1e-6f) {
            m_mass = 1.0f / m_mass;
        }

        // Warm start
        Vec2 impulse = m_normal * m_impulseSum;
        m_bodyA->applyImpulseAtPoint(-impulse, pA);
        m_bodyB->applyImpulseAtPoint(impulse, pB);
    }

    void DistanceConstraint::solve() {
        if (!m_enabled) return;

        Vec2 pA = m_bodyA->getPosition() + m_rA;
        Vec2 pB = m_bodyB->getPosition() + m_rB;

        Vec2 delta = pB - pA;
        float currentDist = delta.length();
        float error = currentDist - m_distance;

        Vec2 vA = m_bodyA->getVelocityAtPoint(pA);
        Vec2 vB = m_bodyB->getVelocityAtPoint(pB);
        Vec2 relativeVel = vB - vA;

        // Compute impulse magnitude
        // REMEMBER: impulse = -(velocity_error + bias) * effective_mass
        float velocityError = relativeVel.dot(m_normal);
        float bias = error * 0.2f;
        float lambda = -(velocityError + bias) * m_mass;
        m_impulseSum += lambda;

        // Apply impulse to both bodies
        Vec2 impulse = m_normal * lambda;
        m_bodyA->applyImpulseAtPoint(-impulse, pA);
        m_bodyB->applyImpulseAtPoint(impulse, pB);
    }
}
