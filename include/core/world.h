#pragma once

#include "core/rigid_body.h"
#include "collision/collision_detector.h"
#include "collision/spatial_hash.h"
#include "solver/constraint_solver.h"
#include "math/vec2.h"
#include <vector>
#include <memory>

namespace parallax {

/**
 * @brief Physics world that manages all bodies and simulation
 * 
 * My simulation pipeline:
 * 1. Apply forces (gravity)
 * 2. Broad-phase collision (spatial hash)
 * 3. Narrow-phase collision (SAT)
 * 4. Solve constraints (iterative)
 * 5. Integrate velocities
 * 6. Update positions
 */

class World {
public:
    World();

    void step(float dt);

    void addBody(RigidBody* body);
    void removeBody(RigidBody* body);
    void clearBodies();

    std::vector<RigidBody*>& getBodies() {
        return m_bodies;
    }

    const std::vector<RigidBody*>& getBodies() const {
        return m_bodies;
    }

    //constraints needed for physics world
    void addConstraint(std::shared_ptr<Constraint> constraint);
    void removeConstraint(std::shared_ptr<Constraint> constraint);
    void clearConstraints();

    void setGravity(const Vec2& gravity) {
        m_gravity = gravity;
    }

    Vec2 getGravity() const {
        return m_gravity;
    }

    void setIterations(int iterations){
        m_iterations = iterations;
    }

    int getIterations() const {
        return m_iterations;
    }

    void setSpatialHashCellSize(float size);

    RigidBody* queryPoint(const Vec2& point);
    std::vector<RigidBody*> queryRegion(const Vec2& min, const Vec2& max);

    size_t getBodyCount() const {
        return m_bodies.size();
    }

    size_t getConstraintCount() const {
        return m_constraints.size();
    }

    size_t getContactCount() const {
        return m_constacts.size();
    }

private:
    std::vector<RigidBody*> m_bodies;

    Vec2 m_gravity;

    SpatialHash m_spatialHash;
    std::vector<Manifold> m_contacts;

    ConstraintSolver m_solver;
    void applyForces(float dt);
    void detectCollisions();
    void solveConstraints(float dt);
    void integrate(float dt);
}
}