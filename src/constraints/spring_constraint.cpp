#include "constraints/spring_constraint.h"
#include <cmath>

namespace parallax {

SpringConstraint::SpringConstraint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec2& anchorA,
    const Vec2& anchorB,
    float restLength,
    float stiffness,
    float damping)
    : Constraint(bodyA, bodyB)
    , m_localAnchorA(anchorA)
    , m_localAnchorB(anchorB)
    , m_restLength(restLength)
    , m_stiffness(stiffness)
    , m_damping(damping)
{}

void SpringConstraint::prepare(float dt) {
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
}

void SpringConstraint::solve() {
    if (!m_enabled) return;

    Vec2 pA = m_bodyA->getPosition() + m_rA;
    Vec2 pB = m_bodyB->getPosition() + m_rB;

    Vec2 delta = pB - pA;
    float currentDist = delta.length();
    float error = currentDist - m_restLength;
    
    Vec2 vA = m_bodyA->getVelocityAtPoint(pA);
    Vec2 vB = m_bodyB->getVelocityAtPoint(pB);
    Vec2 relativeVel = vB - vA;
    float velocityAlongNormal = relativeVel.dot(m_normal);

    float springForce = m_stiffness * error;
    float dampingForce = m_damping * velocityAlongNormal;
    float lambda = -(springForce + dampingForce) * m_mass;

    Vec2 impulse = m_normal * lambda;
    m_bodyA->applyImpulseAtPoint(-impulse, pA);
    m_bodyB->applyImpulseAtPoint(impulse, pB);
}