/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef LATERAL_CONTROL_H
#define LATERAL_CONTROL_H

#include <vector>
#include "trainersimTest.h"

class LateralControl : public TrainersimTest
{
public:

    LateralControl() {
        m_quad->setControlMode(CM_ANGLE_MODE);
        m_currentGoalIndex = 1;
        createGoal();
        moveGoal();
    }

    virtual void moveGoal()
    {
        std::vector<int> inds;
        for (int i = 0; i < 3; i++)
            if ( i != m_currentGoalIndex )
                inds.push_back(i);
        int rnd = rand() % inds.size();
        m_currentGoalIndex = inds[rnd];
        m_goalBody->SetTransform( b2Vec2(-6 + 6 * m_currentGoalIndex, 5 ), 0 );
    }

    virtual bool allowThrottleControl() { return false; }
    virtual bool allowModeSwitching() { return false; }
    virtual bool allowExpo() { return true; }
    virtual bool allowRate() { return true; }

    void Step(Settings* settings)
    {
        float angle = fabsf( m_quad->getBody()->GetAngle() );

        float verticalForceToCounterGravity = m_quad->getBody()->GetMass() * -m_world->GetGravity().y;
        float thrustDirForceToCounterGravity = verticalForceToCounterGravity / cosf(angle);
        float maxPower = m_quad->getMaxPower();
        float throttle = thrustDirForceToCounterGravity / maxPower;

        // this calculation is not exact, need to correct vertical wandering over time
        float heightError = 5 - m_quad->getBody()->GetPosition().y;
        float correction = heightError * 0.01;

        m_quad->setThrottle(throttle * (1+correction));

        TrainersimTest::Step(settings);
    }

    static Test* Create()
    {
        return new LateralControl;
    }

    int m_currentGoalIndex;
};

#endif
