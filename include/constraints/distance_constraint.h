#pragma once

#include "constraints/constraint.h"
#include "math/vec2.h"

namespace parallax {
    /*
    * @brief Constraint that keeps two bodies a fixed distance apart
    */
    class DistanceConstraint : public Constraint {
        public:
            DistanceConstraint(RigidBody* bodyA, RigidBody* bodyB, const Vec2& anchorA, const Vec2& anchorB, float distance);

            void prepare(float dt) override;
            void solve() override;

            Vec2 getAnchorA() const {
                return m_localAnchorA;
            }

            Vec2 getAnchorB() const{
                return m_localAnchorB;
            }

            void setDistance(float distance) {
                m_distance = distance;
            }
        
        private:
            Vec2 m_localAnchorA;
            Vec2 m_localAnchorB;

            float m_distance;

            Vec2 m_rA;
            Vec2 m_rB;
            Vec2 m_normal;
            float m_mass;

            float m_impulseSum;
    };
}