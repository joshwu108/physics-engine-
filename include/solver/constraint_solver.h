#pragma once

#include "constraints/constraint.h"
#include "collision/collision_detector.h"
#include "math/vec2.h"
#include <vector>
#include <memory>

namespace parallax {
/**
 * @brief Constraint solver that uses the Gauss-Seidel method to solve constraints
 */
class ConstraintSolver {
public:
    ConstraintSolver();

    void solve(float dt, int iterations=10);

    void addConstraint(std::shared_ptr<Constraint> constraint);
    void removeConstraint(std::shared_ptr<Constraint> constraint);
    void clearConstraints();

    void setContacts(const std::vector<Manifold>& manifolds);

    void setIterations(int iterations) {
        m_iterations = iterations;
    }

    int getIterations() const {
        return m_iterations;
    }

    size_t getConstraintCount() const {
        return m_constraints.size();
    }
    size_t getContactCount() const {
        return m_contactConstraints.size();
    }

private:
    std::vector<std::shared_ptr<Constraint>> m_constraints;
    std::vector<ContactConstraint> m_contactConstraints;

    int m_iterations;
    /**
     * @brief Contact constraints for solving contacts
     */
    class ContactConstraint {
    public:
        ContactConstraint(const Contact& contact);

        void prepare(float dt);
        void solve();

    private:
        RigidBody* m_bodyA;
        RigidBody* m_bodyB;

        Vec2 m_position; //collision point
        Vec2 m_normal; 
        float m_penetration; //overlap betweeen the bodies

        float m_restitution;
        float m_friction;

        Vec2 m_rA, m_rB;
        float m_normalMass;
        float m_tangentMass;

        float m_normalImpulse;
        float m_tangentImpulse;
    };
};

}