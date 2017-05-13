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

#ifndef QUAD_H
#define QUAD_H

#include <Box2D/Box2D.h>
#include "PIDController.h"

#define	RAND_LIMIT	32767

enum _controlMode {
    CM_ANGLE_MODE,
    CM_RATE_MODE
};

inline float32 randomFloatInRange(float32 lo, float32 hi)
{
    float32 r = (float32)(rand() & (RAND_LIMIT));
    r /= RAND_LIMIT;
    r = (hi - lo) * r + lo;
    return r;
}

class Quad
{
    b2Body* m_body;
    b2Fixture* m_centralFixture;

    static int m_numCells;
    float m_maxPower;

    float m_throttle;
    float m_rollInput;
    float m_rateInput;
    float m_angleInput;

    float m_leftThrust;
    float m_rightThrust;

    PIDController m_ratePID;
    PIDController m_anglePID;

    static int m_controlMode;
    bool m_controllingAngle; // false implies rate mode

    static float m_rate;
    static float m_expo;

    void setDesiredRate(float r);
    void setDesiredAngle(float a);

public:

    static float thrust2S;
    static float thrust3S;
    static float thrust4S;

    Quad(b2World* world, b2Body *ground, b2Vec2 startPosition);
    void setFixedRotation(bool b);
    void setBatteryCells(int s);
    void updateBatteryCells();
    void step();
    void controlAngle();
    void controlRate();
    void renderThrust();

    void setControlMode(int m);
    void setRoll(float val);

    int getControlMode() { return m_controlMode; }
    float getExpo() { return m_expo; }
    void setExpo(float e) { m_expo = e; }
    float getRate() { return m_rate; }
    void setRate(float e) { m_rate = e; }

    b2Body* getBody() { return m_body; }
    b2Fixture* getCentralFixture() { return m_centralFixture; }

    int getBatteryCells() { return m_numCells; }
    void setThrottle(float t) {m_throttle = b2Clamp(t, 0.0f, 1.0f);}
    float getThrottle() { return m_throttle; }

    float getMaxPower() { return m_maxPower; }
    float getLeftThrust() { return m_leftThrust; }
    float getRightThrust() { return m_rightThrust; }

    float getRollInput() { return m_rollInput; }
};

#endif
