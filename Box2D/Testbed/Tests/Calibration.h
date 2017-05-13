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

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdio.h>
#include <fstream>
#include <Box2D/Box2D.h>
#include "freeglut/GL/glut.h"
#include "../Framework/Test.h"
#include "../Framework/json/json.h"
#include "../libstem_gamepad/include/gamepad/Gamepad.h"

class Calibration : public Test
{
public:

    Calibration() {

        //m_throttleChannel = 999;
        //m_rollChannel = 999;

        m_savingTimer = 0;
        m_doingStartup = true;

        b2BodyDef bd;
        b2Body* ground = m_world->CreateBody(&bd);

        //b2PolygonShape shape;
        //shape.SetAsEdge(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
        //ground->CreateFixture(&shape, 0.0f);

        m_device = NULL;
        int numDevices = Gamepad_numDevices();
        printf("Found %d devices.\n", numDevices);
        if ( numDevices > 0 ) {
            m_device = Gamepad_deviceAtIndex(0);
            printf("Using %s\n", m_device->description);

            printf("    numAxes: %d\n", m_device->numAxes);

            float lx = -5;
            float spacing = 10 / (float)(m_device->numAxes-1);

            for (int i = 0; i < m_device->numAxes; i++) {
                b2CircleShape shape;
                shape.m_radius = 0.2;
                shape.m_p.Set( lx + i * spacing, 4.5 );

                b2FixtureDef fd;
                fd.shape = &shape;
                m_axisFixtures[i] = ground->CreateFixture(&fd);
            }

            for (int i = 0; i < m_device->numAxes; i++) {
                b2CircleShape shape;
                shape.m_radius = 0.2;
                shape.m_p.Set( lx + i * spacing, 8.5 );

                b2FixtureDef fd;
                fd.shape = &shape;
                m_reverseAxisFixtures[i] = ground->CreateFixture(&fd);
            }

            // throttle plug
            {
                bd.type = b2_dynamicBody;
                bd.position.Set(-0.55,2);
                if ( s_throttleChannel != 999 ) {
                    bd.position.Set(lx + (abs(s_throttleChannel)-1) * spacing, 4.35);
                    if ( s_throttleChannel < 0 )
                        bd.position.y += 4;
                }
                bd.fixedRotation = true;
                bd.linearDamping = 100;
                m_throttleBody = m_world->CreateBody(&bd);

                b2CircleShape shape;
                shape.m_radius = 0.2;

                b2FixtureDef fd;
                fd.isSensor = true;
                fd.shape = &shape;

                m_throttleBody->CreateFixture(&fd);
            }

            // roll plug
            {
                bd.type = b2_dynamicBody;
                bd.position.Set( 0.55,2);
                if ( s_throttleChannel != 999 ) {
                    bd.position.Set(lx + (abs(s_rollChannel)-1) * spacing, 4.35);
                    if ( s_rollChannel < 0 )
                        bd.position.y += 4;
                }
                bd.fixedRotation = true;
                bd.linearDamping = 100;
                m_rollBody = m_world->CreateBody(&bd);

                b2CircleShape shape;
                shape.m_radius = 0.2;

                b2FixtureDef fd;
                fd.isSensor = true;
                fd.shape = &shape;

                m_rollBody->CreateFixture(&fd);
            }
        }

        fflush(stdout);

        // consume first beginContact callbacks
        m_world->Step(1/60.0f,8,3);

        updateThrottle();
        updateRoll();

        m_doingStartup = false;
    }

    virtual void KeyboardUp(unsigned char key)
    {
        switch (key) {
        case 's':
            saveSettings();
            break;
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

    int getChannel(b2Fixture* f)
    {
        if ( !m_device )
            return 999;

        for (int i = 0; i < m_device->numAxes; i++) {
            if ( m_axisFixtures[i] == f )
                return i + 1;
        }

        for (int i = 0; i < m_device->numAxes; i++) {
            if ( m_reverseAxisFixtures[i] == f )
                return -(i + 1);
        }

        return 999;
    }

    void updateThrottle()
    {
        float tval = getCurrentThrottleOutput(m_device);
        s_minThrottle = b2Min(tval, s_minThrottle);
        s_maxThrottle = b2Max(tval, s_maxThrottle);
    }

    void updateRoll()
    {
        float rval = getCurrentRollOutput(m_device);
        s_minRoll = b2Min(rval, s_minRoll);
        s_maxRoll = b2Max(rval, s_maxRoll);
    }

    virtual void BeginContact(b2Contact* contact)
    {
        if ( m_doingStartup )
            return;

        if ( contact->GetFixtureA()->GetBody() == m_throttleBody && contact->GetFixtureB()->GetBody()->GetType() == b2_staticBody ) {
            s_throttleChannel = getChannel(contact->GetFixtureB());
            s_minThrottle = 999;
            s_maxThrottle = -999;
            updateThrottle();
        }
        if ( contact->GetFixtureB()->GetBody() == m_throttleBody && contact->GetFixtureA()->GetBody()->GetType() == b2_staticBody ) {
            s_throttleChannel = getChannel(contact->GetFixtureA());
            s_minThrottle = 999;
            s_maxThrottle = -999;
            updateThrottle();
        }

        if ( contact->GetFixtureA()->GetBody() == m_rollBody && contact->GetFixtureB()->GetBody()->GetType() == b2_staticBody ) {
            s_rollChannel = getChannel(contact->GetFixtureB());
            s_minRoll = 999;
            s_maxRoll = -999;
            updateRoll();
        }
        if ( contact->GetFixtureB()->GetBody() == m_rollBody && contact->GetFixtureA()->GetBody()->GetType() == b2_staticBody ) {
            s_rollChannel = getChannel(contact->GetFixtureA());
            s_minRoll = 999;
            s_maxRoll = -999;
            updateRoll();
        }
    }

    virtual void EndContact(b2Contact* contact)
    {
        if ( contact->GetFixtureA()->GetBody() == m_throttleBody && contact->GetFixtureB()->GetBody()->GetType() == b2_staticBody )
            s_throttleChannel = 999;
        if ( contact->GetFixtureB()->GetBody() == m_throttleBody && contact->GetFixtureA()->GetBody()->GetType() == b2_staticBody )
            s_throttleChannel = 999;

        if ( contact->GetFixtureA()->GetBody() == m_rollBody && contact->GetFixtureB()->GetBody()->GetType() == b2_staticBody )
            s_rollChannel = 999;
        if ( contact->GetFixtureB()->GetBody() == m_rollBody && contact->GetFixtureA()->GetBody()->GetType() == b2_staticBody )
            s_rollChannel = 999;
    }

#define SETTINGS_FILE "quadnoobtrainer.json"

    void saveSettings()
    {
        if ( s_throttleChannel == 999 || s_rollChannel == 999 )
            return;

        Json::Value val;
        val["throttleChannel"] = s_throttleChannel;
        val["rollChannel"] = s_rollChannel;
        val["minThrottle"] = s_minThrottle;
        val["maxThrottle"] = s_maxThrottle;
        val["minRoll"] = s_minRoll;
        val["maxRoll"] = s_maxRoll;

        std::ofstream ofs;
        ofs.open(SETTINGS_FILE, std::ios::out);

        Json::StyledStreamWriter writer("  ");
        writer.write(ofs, val);

        ofs.close();

        m_savingTimer = 120; // two seconds
    }

    static void loadSettings() {

        std::ifstream ifs;
        ifs.open(SETTINGS_FILE, std::ios::in);
        if (!ifs)
            return;

        Json::Value val;
        Json::Reader reader;
        if ( ! reader.parse(ifs, val) )
        {
            ifs.close();
            return;
        }
        ifs.close();

        s_throttleChannel = val.get("throttleChannel",999).asInt();
        s_rollChannel = val.get("rollChannel",999).asInt();
        s_minThrottle = val.get("minThrottle",0).asFloat();
        s_maxThrottle = val.get("maxThrottle",0).asFloat();
        s_minRoll = val.get("minRoll",0).asFloat();
        s_maxRoll = val.get("maxRoll",0).asFloat();
    }

    static float getCurrentThrottleOutput(Gamepad_device* device)
    {
        float tval = 0;

        if ( device ) {
            if ( s_throttleChannel != 999 ) {
                int channelInd = abs(s_throttleChannel) - 1;
                tval = device->axisStates[channelInd];
                if ( s_throttleChannel < 0 )
                    tval *= -1;
            }
        }

        return tval;
    }

    static float getCurrentRollOutput(Gamepad_device* device)
    {
        float rval = 0;

        if ( device ) {
            if ( s_rollChannel != 999 ) {
                int channelInd = abs(s_rollChannel) - 1;
                rval = device->axisStates[channelInd];
                if ( s_rollChannel < 0 )
                    rval *= -1;
            }
        }

        return rval;
    }

    virtual void Step(Settings* settings)
    {
        if ( m_device ) {
            glColor3f(1,0.5,0.5);
            glBegin(GL_LINES);
            glVertex2f( 0.55, 1.5 );
            glVertex2f( m_rollBody->GetPosition().x, m_rollBody->GetPosition().y );
            glVertex2f( -0.55, 1.5 );
            glVertex2f( m_throttleBody->GetPosition().x, m_throttleBody->GetPosition().y );
            glEnd();

            updateThrottle();
            updateRoll();
        }

        Test::Step(settings);


        if ( ! m_device ) {
            m_debugDraw.DrawString(5, m_textLine, "No device found.");
            m_textLine += 15;
            m_debugDraw.DrawString(5, m_textLine, "Plug in a gamepad or trainer port cable and restart the program.");
            m_textLine += 15;
            return;
        }


        float lx = -5;
        float spacing = 10 / (float)(m_device->numAxes-1);

        glColor3f(0.5,0.5,0.5);
        glBegin(GL_LINES);
        for (int i = 0; i < m_device->numAxes; i++) {
            glVertex2f(lx + i * spacing, 5);
            glVertex2f(lx + i * spacing, 8);
        }
        glEnd();

        glEnable(GL_POINT_SMOOTH);
        glPointSize(15);
        glColor3f(0,1,0);
        glBegin(GL_POINTS);
        for (int i = 0; i < m_device->numAxes; i++) {
            glVertex2f(lx + i * spacing, 6.5 + m_device->axisStates[i] * 1.5);
        }
        glEnd();

        m_debugDraw.DrawString(5, m_textLine, "Using %s ", m_device->description);
        m_textLine += 15;

        if ( s_throttleChannel != 999 ) {
            m_debugDraw.DrawString(5, m_textLine, "Throttle uses channel %d ", s_throttleChannel);
            m_textLine += 15;
        }
        if ( s_rollChannel != 999 ) {
            m_debugDraw.DrawString(5, m_textLine, "Roll uses channel %d ", s_rollChannel);
            m_textLine += 15;
        }

        if ( m_savingTimer-- > 0 ) {
            m_debugDraw.DrawString(5, m_textLine, "Saved settings");
            m_textLine += 15;
        }
        else if ( s_throttleChannel != 999 && s_rollChannel != 999 ) {
            m_debugDraw.DrawString(5, m_textLine, "Press S to save settings.");
            m_textLine += 15;
        }

        float tval = 0;
        float tdenom = s_maxThrottle - s_minThrottle;
        if ( tdenom != 0 )
            tval = (getCurrentThrottleOutput(m_device) - s_minThrottle) / tdenom;
        tval = tval * 2 - 1;

        float rval = 0;
        float rdenom = s_maxRoll - s_minRoll;
        if ( rdenom != 0 )
            rval = (getCurrentRollOutput(m_device) - s_minRoll) / rdenom;
        rval = rval * 2 - 1;

        glColor3f(0,0.25,0);
        glBegin(GL_LINES);
        glVertex2f(-0.55,0.5);
        glVertex2f(-0.55,1.5);
        glVertex2f( 0.05,1);
        glVertex2f( 1.05,1);
        glEnd();

        renderControlInputs(m_device != NULL, 0, tval, b2Vec2(-0.55,1));
        renderControlInputs(m_device != NULL, rval, 0, b2Vec2( 0.55,1));
    }

    static Test* Create()
    {
        return new Calibration;
    }

    Gamepad_device* m_device;
    b2Fixture* m_axisFixtures[16];
    b2Fixture* m_reverseAxisFixtures[16];
    b2Body* m_throttleBody;
    b2Body* m_rollBody;

    bool m_doingStartup;

    static int s_throttleChannel;
    static int s_rollChannel;
    static float s_minThrottle;
    static float s_maxThrottle;
    static float s_minRoll;
    static float s_maxRoll;

    int m_savingTimer;
};

#endif
