#pragma once

#include "core/rigid_body.h"
#include "math/vec2.h"

namespace parallax {
    /**
     * @brief Base class for constraints that limit how bodies interact with each other
     */
    class Constraint {
        public:
            virtual ~Constraint() = default;

            virtual void prepare(float dt) = 0;
            virtual void solve() = 0;

            RigidBody* getBodyA() const { return m_bodyA; }
            RigidBody* getBodyB() const { return m_bodyB; }
            
            void setEnabled(bool enabled) { 
                m_enabled = enabled; 
            }

            bool isEnabled() const {
                return m_enabled;
            }

        protected:
            Constraint(RigidBody* bodyA, RigidBody* bodyB)
                : m_bodyA(bodyA)
                , m_bodyB(bodyB)
                , m_enabled(true)
            {}

            RigidBody* m_bodyA;
            RigidBody* m_bodyB;
            bool m_enabled;
    };
}
