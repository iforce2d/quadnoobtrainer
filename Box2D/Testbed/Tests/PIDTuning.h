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

#ifndef PID_TUNING_H
#define PID_TUNING_H

#include <vector>
#include "PIDController.h"

class PIDTuning : public Test
{
public:

    PIDTuning()
    {
        b2BodyDef bd;
        m_ground = m_world->CreateBody(&bd);

        b2PolygonShape shape;
        shape.SetAsEdge(b2Vec2(-40.0f, 1.98f), b2Vec2(40.0f, 1.98f));
        m_ground->CreateFixture(&shape, 0.0f);
        shape.SetAsEdge(b2Vec2(-0.5f, 1.9f), b2Vec2(-0.5f, 1.6f));
        m_ground->CreateFixture(&shape, 0.0f);
        shape.SetAsEdge(b2Vec2(0.5f, 1.9f), b2Vec2(0.5f, 1.6f));
        m_ground->CreateFixture(&shape, 0.0f);

        bd.type = b2_dynamicBody;
        bd.position.Set(0,2.3);

        m_sledBody = m_world->CreateBody(&bd);

        shape.SetAsBox(0.5,0.3);

        b2FixtureDef fd;
        fd.density = 1;
        fd.shape = &shape;
        m_sledBody->CreateFixture(&fd);

        b2PrismaticJointDef jd;
        jd.bodyA = m_ground;
        jd.bodyB = m_sledBody;
        jd.localAnchorA = m_sledBody->GetPosition();
        jd.localAnchorB.Set(0,0);
        jd.localAxis1.Set(1,0);
        m_world->CreateJoint(&jd);

        //m_pid.setGains(5,0,3);
        m_pid.reset();
    }

    virtual void Keyboard(unsigned char key)
    {
        Test::Keyboard(key);

        float factor = 1.05f;
        float newval = 0;

        switch (key) {
        case 'a':
            newval = m_pid.getP() / factor;
            if ( newval <= 0.05 )
                newval = 0;
            m_pid.setP( newval );
            break;
        case 'q':
            newval = m_pid.getP() * factor;
            if ( newval <= 0 )
                newval = 0.05;
            m_pid.setP( newval );
            break;
        case 's':
            newval = m_pid.getI() / factor;
            if ( newval <= 0.05 )
                newval = 0;
            m_pid.setI( newval );
            break;
        case 'w':
            newval = m_pid.getI() * factor;
            if ( newval <= 0 )
                newval = 0.05;
            m_pid.setI( newval );
            break;
        case 'd':
            newval = m_pid.getD() / factor;
            if ( newval <= 0.05 )
                newval = 0;
            m_pid.setD( newval );
            break;
        case 'e':
            newval = m_pid.getD() * factor;
            if ( newval <= 0 )
                newval = 0.05;
            m_pid.setD( newval );
            break;
        }
    }

    virtual void Step(Settings* settings)
    {
        m_pid.setSetPointAndCurrentPoint( 0, m_ground->GetLocalPoint(m_sledBody->GetPosition()).x );
        m_pid.step( 1/60.0f );
        m_thrust = m_pid.getOutput();
        m_thrust = b2Clamp(m_thrust,-6.0f,6.0f);

        m_sledBody->ApplyForce( b2Vec2(m_thrust,0), m_sledBody->GetPosition() );

        Test::Step(settings);

        renderThrust();

        m_debugDraw.DrawString(5, m_textLine, "Press A/Q to change proportional value (currently P = %0.2f)", m_pid.getP());
        m_textLine += 15;
        m_debugDraw.DrawString(5, m_textLine, "Press S/W to change integral value     (currently I = %0.2f)", m_pid.getI());
        m_textLine += 15;
        m_debugDraw.DrawString(5, m_textLine, "Press D/E to change derivative value   (currently D = %0.2f)", m_pid.getD());
        m_textLine += 15;
    }

    void renderThrust()
    {
        int numThrustLines = sqrtf(fabsf(m_thrust)) * 16;

        glEnable(GL_BLEND);
        glColor4f(1,1,0.75,0.5);
        glBegin(GL_LINES);

        for ( int i = 0; i < numThrustLines; i++ ) {
            float length = randomFloatInRange(0, m_thrust);
            float angle = randomFloatInRange(-0.1, 0.1);
            b2Vec2 line = b2Vec2(-length,0);
            b2Mat22 rot = b2Mat22(angle);
            line = b2Mul(rot,line);
            b2Vec2 start = b2Vec2(m_thrust>0?-0.5:0.5,0);
            start.y += randomFloatInRange(-0.05,0.05);
            b2Vec2 end = start + line;
            start = m_sledBody->GetWorldPoint(start);
            end = m_sledBody->GetWorldPoint(end);
            glVertex2f( start.x, start.y );
            glVertex2f( end.x, end.y );
        }

        glEnd();
    }

    static Test* Create()
    {
        return new PIDTuning;
    }

    b2Body* m_ground;
    b2Body* m_sledBody;
    static PIDController m_pid;
    float m_thrust;
};

PIDController PIDTuning::m_pid;

#endif
