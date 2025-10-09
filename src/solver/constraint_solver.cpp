#include "solver/constraint_solver.h"
#include "math/vec2.h"
#include <algorithm>


namespace parallax {

ConstraintSolver::ConstraintSolver()
    : m_iterations(10)
{}

void ConstraintSolver::solve(float dt, int iterations) {
    if(iterations > 0){
        m_iterations = iterations;
    }

    //prepare constraints
    for(auto& constraint : m_constraints) {
        if(constraint) {
            constraint->prepare(dt);
        }
    }

    for(auto& contact: m_contactConstraints) {
        contact.prepare(dt);
    }

    for(int i = 0; i < m_iterations; ++i) {
        for(auto& constraint: m_constraints) {
            if(constraint) {
                constraint->solve();
            }
        }

        for(auto& contact: m_contactConstraints) {
            contact.solve();
        }
    }
}

void ConstraintSolver::addConstraint(std::shared_ptr<Constraint> constraint) {
    if (constraint) {
        m_constraints.push_back(constraint);
    }
}

void ConstraintSolver::removeConstraint(std::shared_ptr<Constraint> constraint) {
    auto it = std::find(m_constraints.begin(), m_constraints.end(), constraint);
    if(it != m_constraints.end()) {
        m_constraints.erase(it);
    }
}

void ConstraintSolver::clearConstraints() {
    m_constraints.clear();
}

void ConstraintSolver::setContacts(const std::vector<Manifold>& manifolds) {
    m_contactConstraints.clear();
    for(const auto& manifold: manifolds) {
        for(const auto& contact: manifold.contacts) {
            m_contactConstraints.emplace_back(ContactConstraint(contact));
        }
    }
}

ConstraintSolver::ContactConstraint::ContactConstraint(const Contact& contact)
    : m_bodyA(contact.bodyA)
    , m_bodyB(contact.bodyB)
    , m_position(contact.position)
    , m_normal(contact.normal)
    , m_penetration(contact.penetration)
    , m_restitution(contact.restitution)
    , m_friction(contact.friction)
    , m_normalImpulse(0.0f)
    , m_tangentImpulse(0.0f)
{}

void ConstraintSolver::ContactConstraint::prepare(float dt){
    m_rA = m_position - m_bodyA->getPosition();
    m_rB = m_position - m_bodyB->getPosition();

    float rnA = m_rA.cross(m_normal);
    float rnB = m_rB.cross(m_normal);

    float kNormal = m_bodyA->getInvMass() + m_bodyB->getInvMass() + rnA * rnA * m_bodyA->getInvInertia() + rnB * rnB * m_bodyB->getInvInertia();
    m_normalMass = (kNormal > 1e-6f) ? (1.0f / kNormal) : 0.0f;

    Vec2 tangent = m_normal.perpendicular();
    float rtA = m_rA.cross(tangent);
    float rtB = m_rB.cross(tangent);

    float kTangent = m_bodyA->getInvMass() + m_bodyB->getInvMass() + rtA * rtA * m_bodyA->getInvInertia() + rtB * rtB * m_bodyB->getInvInertia();
    m_tangentMass = (kTangent > 1e-6f) ? (1.0f / kTangent) : 0.0f;

    Vec2 impulse = m_normal * m_normalImpulse + tangent * m_tangentImpulse;
    m_bodyA->applyImpulseAtPoint(-impulse, m_position);
    m_bodyB->applyImpulseAtPoint(impulse, m_position);
}

void ConstraintSolver::ContactConstraint::solve(){
    Vec2 vA = m_bodyA->getVelocityAtPoint(m_position);
    Vec2 vB = m_bodyB->getVelocityAtPoint(m_position);
    Vec2 relativeVel = vB - vA;

    float normalVel = relativeVel.dot(m_normal);

    const float baumgarte = 0.2f;
    const float slop = 0.01f;
    float bias = (baumgarte / 0.016f) * std::max(m_penetration - slop, 0.0f);
    float restitutionBias = 0.0f;
    const float restitutionThreshold = 1.0f;
    if (std::abs(normalVel) > restitutionThreshold) {
        restitutionBias = -m_restitution * normalVel;
    }

    float lambda = -(normalVel + bias + restitutionBias) * m_normalMass;
    float oldImpulse = m_normalImpulse;
    m_normalImpulse = std::max(m_normalImpulse + lambda, 0.0f);
    lambda = m_normalImpulse - oldImpulse;

    Vec2 impulse = m_normal * lambda;
    m_bodyA->applyImpulseAtPoint(-impulse, m_position);
    m_bodyB->applyImpulseAtPoint(impulse, m_position);
    relativeVel = vB - vA;

    Vec2 tangent = m_normal.perpendicular();
    float tangentVel = relativeVel.dot(tangent);

    float frictionLambda = -tangentVel * m_tangentMass;
    float maxFriction = m_friction * m_normalImpulse;
    float oldTangentImpulse = m_tangentImpulse;
    m_tangentImpulse = std::clamp(m_tangentImpulse + frictionLambda, -maxFriction, maxFriction);
    frictionLambda = m_tangentImpulse - oldTangentImpulse;
    
    // Apply friction impulse
    Vec2 frictionImpulse = tangent * frictionLambda;
    m_bodyA->applyImpulseAtPoint(-frictionImpulse, m_position);
    m_bodyB->applyImpulseAtPoint(frictionImpulse, m_position);
}
}