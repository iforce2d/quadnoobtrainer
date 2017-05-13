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

#ifndef TRAINERSIM_TEST_H
#define TRAINERSIM_TEST_H

#include "quad.h"

#include "../libstem_gamepad/include/gamepad/Gamepad.h"
#include "../Tests/Calibration.h"

#define STEPS_FOR_GOAL_SUCCESS 180

/*
// gamepad mode 2
#define GAMEPAD_AXIS_YAW        0
#define GAMEPAD_AXIS_THROTTLE   1
#define GAMEPAD_AXIS_ROLL       2
#define GAMEPAD_AXIS_PITCH      3

// FlySky mode 2
#define FLYSKY_AXIS_YAW        4
#define FLYSKY_AXIS_THROTTLE   1
#define FLYSKY_AXIS_ROLL       3
#define FLYSKY_AXIS_PITCH      0
*/

enum _controlDevice {
    CD_KEYBOARD,
    CD_GAMEPAD
};

class TrainersimTest : public Test
{
public:

    TrainersimTest() {
        m_ground = createGround();
        m_quad = new Quad(m_world, m_ground, b2Vec2(0,5));
        m_quad->updateBatteryCells();
        m_quad->setThrottle(0);
        m_quad->setRoll(0);

        m_throttleKeyDown = false;
        m_leftKeyDown = false;
        m_rightKeyDown = false;

        m_goalBody = NULL;
        m_touchingGoal = false;
        m_goalSuccessCounter = 0;
    }

    b2Body* createGround()
    {
        b2BodyDef bd;
        b2Body* ground = m_world->CreateBody(&bd);

        b2PolygonShape shape;
        shape.SetAsEdge(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
        ground->CreateFixture(&shape, 0.0f);

        return ground;
    }

    b2Body* createGoal()
    {
        b2BodyDef bd;
        bd.position.Set(0,-100);
        bd.type = b2_kinematicBody;
        m_goalBody = m_world->CreateBody(&bd);

        b2PolygonShape shape;
        shape.SetAsBox(1,1);

        b2FixtureDef fd;
        fd.isSensor = true;
        fd.shape = &shape;

        m_goalBody->CreateFixture(&fd);

		return m_goalBody;
    }

    virtual void moveGoal() {}

    virtual bool allowThrottleControl() { return true; }
    virtual bool allowLateralControl() { return true; }
    virtual bool allowModeSwitching() { return true; }
    virtual bool allowExpo() { return false; }
    virtual bool allowRate() { return false; }

    virtual void Keyboard(unsigned char key)
    {
        float factor = 0.02f;
        float newval = 0;

        switch (key) {
        case 'b': {
            int nextCells = m_quad->getBatteryCells() + 1;
            if ( nextCells > 4 )
                nextCells = 2;
            m_quad->setBatteryCells( nextCells );
        }
            break;

        case 'm': {
            int nextControlMode = m_quad->getControlMode() + 1;
            if ( nextControlMode > 1 )
                nextControlMode = 0;
            m_quad->setControlMode( nextControlMode );
        }
            break;

        case 'c':
            m_controlDevice = (m_controlDevice + 1) % 2;
            break;

        case 'w':
            m_throttleKeyDown = true;
            break;
        case 'a':
            m_leftKeyDown = true;
            break;
        case 'd':
            m_rightKeyDown = true;
            break;

        /*case 'g':
            newval = m_quad->getRate() - factor*10;
            if ( newval < 0 )
                newval = 0;
            m_quad->setRate( newval );
            break;
        case 't':
            newval = m_quad->getRate() + factor*10;
            if ( newval > 10 )
                newval = 10;
            m_quad->setRate( newval );
            break;*/

        case 'g':
            newval = m_quad->getExpo() - factor;
            if ( newval < 0 )
                newval = 0;
            m_quad->setExpo( newval );
            break;
        case 't':
            newval = m_quad->getExpo() + factor;
            if ( newval > 1 )
                newval = 1;
            m_quad->setExpo( newval );
            break;
        }
    }

    virtual void KeyboardUp(unsigned char key)
    {
        switch (key) {
        case 'w':
            m_throttleKeyDown = false;
            break;
        case 'a':
            m_leftKeyDown = false;
            break;
        case 'd':
            m_rightKeyDown = false;
            break;
        }
    }

    virtual void axisMove(Gamepad_device* device, unsigned int axisID, float value)
    {
        if ( m_controlDevice == CD_KEYBOARD )
            return;

        if ( allowThrottleControl() ) {

			float tval = 0;
			float tdenom = Calibration::s_maxThrottle - Calibration::s_minThrottle;
			if (tdenom != 0)
				tval = (Calibration::getCurrentThrottleOutput(device) - Calibration::s_minThrottle) / tdenom;
			//tval = tval * 2 - 1;
			
			m_quad->setThrottle(tval);
        }

        if ( allowLateralControl() ) {

			float rval = 0;
			float rdenom = Calibration::s_maxRoll - Calibration::s_minRoll;
			if (rdenom != 0)
				rval = (Calibration::getCurrentRollOutput(device) - Calibration::s_minRoll) / rdenom;

            m_quad->setRoll(-(rval * 2 - 1));
        }
    }

    virtual void Step(Settings* settings)
    {
        if ( m_controlDevice == CD_KEYBOARD ) {
            if ( allowThrottleControl() ) {
                if ( m_throttleKeyDown )
                    m_quad->setThrottle(1);
                else
                    m_quad->setThrottle(0);
            }

            if ( allowLateralControl() ) {
                if ( m_leftKeyDown && !m_rightKeyDown )
                    m_quad->setRoll(1);
                else if ( !m_leftKeyDown && m_rightKeyDown )
                    m_quad->setRoll(-1);
                else
                    m_quad->setRoll(0);
            }
        }

        m_quad->step();

        if ( m_touchingGoal ) {
            if ( ++m_goalSuccessCounter > STEPS_FOR_GOAL_SUCCESS ) {
                moveGoal();
            }
        }
        else
            m_goalSuccessCounter = 0;

        if ( m_goalSuccessCounter > 0 ) {
            float percent = m_goalSuccessCounter / (float)STEPS_FOR_GOAL_SUCCESS;

            float w = percent * 2;

            b2Vec2 p0 = m_goalBody->GetPosition() + b2Vec2(-1,1);
            b2Vec2 p1 = p0 + b2Vec2(0,-0.05);
            b2Vec2 p2 = p0 + b2Vec2(w,-0.05);
            b2Vec2 p3 = p0 + b2Vec2(w,0);

            glColor3f(0,1,0);
            glBegin(GL_QUADS);
            glVertex2f(p0.x, p0.y);
            glVertex2f(p1.x, p1.y);
            glVertex2f(p2.x, p2.y);
            glVertex2f(p3.x, p3.y);
            glEnd();
        }

        Test::Step(settings);

        m_quad->renderThrust();
        renderControlInputs(allowThrottleControl(), 0, -1 + 2 * m_quad->getThrottle(), b2Vec2(2.45,8.8));
        renderControlInputs(allowLateralControl(), -m_quad->getRollInput(), 0, b2Vec2(3.55,8.8));

        m_debugDraw.DrawString(5, m_textLine, "Press R to reset");
        m_textLine += 15;
        m_debugDraw.DrawString(5, m_textLine, "Press C to change control device (currently using %s)", m_controlDevice == CD_KEYBOARD ? "keyboard":"gamepad/trainer port");
        m_textLine += 15;
        if ( m_controlDevice == CD_KEYBOARD ) {
            if ( allowThrottleControl() ) {
                m_debugDraw.DrawString(5, m_textLine, "Press W to apply throttle");
                m_textLine += 15;
            }
            if ( allowLateralControl() ) {
                m_debugDraw.DrawString(5, m_textLine, "Press A and D to rotate left/right");
                m_textLine += 15;
            }
        }
        m_debugDraw.DrawString(5, m_textLine, "Press B to change battery (currently %d cells)", m_quad->getBatteryCells());
        m_textLine += 15;
        if ( allowModeSwitching() ) {
            m_debugDraw.DrawString(5, m_textLine, "Press M to change control mode (currently %s)", m_quad->getControlMode()==CM_ANGLE_MODE?"angle mode (auto-level)":"rate mode (acro/gyro)");
            m_textLine += 15;
        }
        /*if ( allowRate() ) {
            m_debugDraw.DrawString(5, m_textLine, "Press G/T to change rate value (currently %0.2f)", m_quad->getRate());
            m_textLine += 15;
        }*/
        if ( allowExpo() && m_controlDevice != CD_KEYBOARD ) {
            m_debugDraw.DrawString(5, m_textLine, "Press G/T to change expo value (currently %0.2f)", m_quad->getExpo());
            m_textLine += 15;
        }
    }

    void renderControlInputs(bool enabled, float x, float y, b2Vec2 pos)
    {
        #define CONTROLBOX_WIDTH    1
        #define CONTROLBOX_HEIGHT   1

        float lx = pos.x - 0.5f * CONTROLBOX_WIDTH;
        float ly = pos.y - 0.5f * CONTROLBOX_HEIGHT;
        float ux = lx + CONTROLBOX_WIDTH;
        float uy = ly + CONTROLBOX_HEIGHT;

        glEnable(GL_POINT_SMOOTH);

        if (enabled)
            glColor3f(0,1,0);
        else
            glColor3f(0,0.25f,0);

        glBegin(GL_LINE_LOOP);
        glVertex2f(lx, ly);
        glVertex2f(ux, ly);
        glVertex2f(ux, uy);
        glVertex2f(lx, uy);
        glEnd();

        if (enabled)
            glColor3f(1,0.5,0.5);
        else
            glColor3f(0.5,0.25,0.25);

        glPointSize(12);
        glBegin(GL_POINTS);
        glVertex2f(pos.x + x * 0.5 * CONTROLBOX_WIDTH, pos.y + y * 0.5 * CONTROLBOX_HEIGHT);
        glEnd();
    }

    virtual void BeginContact(b2Contact* contact)
    {
        b2Fixture* quadFixture = m_quad->getCentralFixture();
        if (( contact->GetFixtureA()->GetBody() == m_goalBody && contact->GetFixtureB() == quadFixture ) ||
            ( contact->GetFixtureB()->GetBody() == m_goalBody && contact->GetFixtureA() == quadFixture ))
            m_touchingGoal = true;
    }

    virtual void EndContact(b2Contact* contact)
    {
        b2Fixture* quadFixture = m_quad->getCentralFixture();
        if (( contact->GetFixtureA()->GetBody() == m_goalBody && contact->GetFixtureB() == quadFixture ) ||
            ( contact->GetFixtureB()->GetBody() == m_goalBody && contact->GetFixtureA() == quadFixture ))
            m_touchingGoal = false;
    }

    b2Body* m_ground;
    Quad* m_quad;

    static int m_controlDevice;

    bool m_throttleKeyDown;
    bool m_leftKeyDown;
    bool m_rightKeyDown;

    b2Body* m_goalBody;
    bool m_touchingGoal;
    int m_goalSuccessCounter;
};

int TrainersimTest::m_controlDevice = CD_KEYBOARD;

#endif
