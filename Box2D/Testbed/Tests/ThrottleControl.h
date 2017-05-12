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

#ifndef THROTTLE_CONTROL_H
#define THROTTLE_CONTROL_H

#include <vector>
#include "trainersimTest.h"

class ThrottleControl : public TrainersimTest
{
public:

    ThrottleControl() {
        m_quad->setControlMode(CM_ANGLE_MODE);
        m_quad->setFixedRotation(true);
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
        m_goalBody->SetTransform( b2Vec2(0, 8 - 3 * m_currentGoalIndex ), 0 );
    }

    virtual bool allowLateralControl() { return false; }
    virtual bool allowModeSwitching() { return false; }

    static Test* Create()
    {
        return new ThrottleControl;
    }

    int m_currentGoalIndex;
};

#endif
