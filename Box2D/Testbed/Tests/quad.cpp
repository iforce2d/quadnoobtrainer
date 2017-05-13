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

#include <stdio.h>
#include "freeglut/GL/glut.h"
#include "quad.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif // !M_PI


#define QUAD_WIDTH  0.3
#define QUAD_HEIGHT 0.04

float Quad::thrust2S = 1.6;
float Quad::thrust3S = 2.4;
float Quad::thrust4S = 3.2;

int Quad::m_numCells = 2;
int Quad::m_controlMode = 0;
float Quad::m_expo = 0;
float Quad::m_rate = 10;

Quad::Quad(b2World* world, b2Body* ground, b2Vec2 startPosition)
{
    b2CircleShape circleShape;
    circleShape.m_radius = 0.1f;

    b2PolygonShape polygonShape;
    polygonShape.SetAsBox(QUAD_WIDTH,QUAD_HEIGHT);

    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position = startPosition;
    bd.linearDamping = 0.1;

    m_body = world->CreateBody(&bd);

    m_centralFixture = m_body->CreateFixture(&circleShape, 1.0f);
    m_body->CreateFixture(&polygonShape, 1.0f);

    m_throttle = 0;
    m_rollInput = 0;
    m_rateInput = 0;
    m_angleInput = 0;

    m_leftThrust = 0;
    m_rightThrust = 0;

    setDesiredAngle(0);

    b2RevoluteJointDef jd;
    jd.bodyA = ground;
    jd.bodyB = m_body;
    jd.localAnchorA = m_body->GetPosition();
    jd.localAnchorB.Set(0,0);
    //world->CreateJoint(&jd);
}

void Quad::setFixedRotation(bool b)
{
    m_body->SetFixedRotation(b);
}

void Quad::setBatteryCells(int s)
{
    m_numCells = s;
    updateBatteryCells();
}

void Quad::updateBatteryCells()
{
    switch (m_numCells) {
    case 4:
        m_maxPower = thrust4S;
        m_ratePID.setGains(6,0,0);
        m_anglePID.setGains(40,0,1.25);
        break;
    case 3:
        m_maxPower = thrust3S;
        m_ratePID.setGains(6,0,0);
        m_anglePID.setGains(80,0,2.5);
        break;
    case 2:
    default :
        m_maxPower = thrust2S;
        m_ratePID.setGains(6,0,0);
        m_anglePID.setGains(120,0,5);
        break;
    }
}

void Quad::step()
{
    b2Vec2 thrustDirection = m_body->GetWorldVector(b2Vec2(0,1));

    if ( m_controllingAngle )
        controlAngle();

    controlRate();

    b2Vec2 leftMotorPos = m_body->GetWorldPoint( b2Vec2(-QUAD_WIDTH+0.05,0) );
    b2Vec2 rightMotorPos = m_body->GetWorldPoint( b2Vec2(QUAD_WIDTH-0.05,0) );

    float currentRate = m_body->GetAngularVelocity();
    float rateError = fabsf(m_rateInput - currentRate);

    m_body->ApplyForce( m_leftThrust * m_maxPower * thrustDirection, leftMotorPos );
    m_body->ApplyForce( m_rightThrust * m_maxPower * thrustDirection, rightMotorPos );
}

void Quad::controlAngle()
{
    float currentAngle = m_body->GetAngle();
    while ( currentAngle > M_PI )
        currentAngle -= 2 * M_PI;
    while ( currentAngle < -M_PI )
        currentAngle += 2 * M_PI;
    m_anglePID.setSetPointAndCurrentPoint( m_angleInput, currentAngle );
    m_anglePID.step( 1/60.0f );
    m_rateInput = m_anglePID.getOutput();
}

void Quad::controlRate()
{
    float currentRate = m_body->GetAngularVelocity();
    m_ratePID.setSetPointAndCurrentPoint( m_rateInput, currentRate );
    m_ratePID.step( 1/60.0f );
    float split = m_ratePID.getOutput();
    //printf("split %f\n", split); fflush(stdout);
    split = b2Clamp(split, -100.0f, 100.0f);
    split /= 200.0f;

    m_leftThrust = 0.5 - split;
    m_rightThrust = split - -0.5;

    m_leftThrust *= m_throttle;
    m_rightThrust *= m_throttle;
}

void Quad::renderThrust() {
    b2Vec2 quadPos = m_body->GetPosition();

    int numThrustLines = sqrtf(m_throttle) * 16;

    glEnable(GL_BLEND);
    glColor4f(1,1,0.75,0.5);
    glBegin(GL_LINES);

    for ( int i = 0; i < numThrustLines; i++ ) {
        float length = randomFloatInRange(0, m_leftThrust);
        float angle = randomFloatInRange(-0.1, 0.1);
        b2Vec2 line = b2Vec2(0,-length);
        b2Mat22 rot = b2Mat22(angle);
        line = b2Mul(rot,line);
        b2Vec2 start = b2Vec2(-QUAD_WIDTH+0.05,-QUAD_HEIGHT);
        start.x += randomFloatInRange(-0.05,0.05);
        b2Vec2 end = start + line;
        start = m_body->GetWorldPoint(start);
        end = m_body->GetWorldPoint(end);
        glVertex2f( start.x, start.y );
        glVertex2f( end.x, end.y );
    }

    for ( int i = 0; i < numThrustLines; i++ ) {
        float length = randomFloatInRange(0, m_rightThrust);
        float angle = randomFloatInRange(-0.1, 0.1);
        b2Vec2 line = b2Vec2(0,-length);
        b2Mat22 rot = b2Mat22(angle);
        line = b2Mul(rot,line);
        b2Vec2 start = b2Vec2(QUAD_WIDTH-0.05,-QUAD_HEIGHT);
        start.x += randomFloatInRange(-0.05,0.05);
        b2Vec2 end = start + line;
        start = m_body->GetWorldPoint(start);
        end = m_body->GetWorldPoint(end);
        glVertex2f( start.x, start.y );
        glVertex2f( end.x, end.y );
    }

    glEnd();
}

void Quad::setControlMode(int m)
{
    m_controlMode = m;
}

void Quad::setRoll(float val)
{
    m_rollInput = val;
    if ( m_controlMode == CM_ANGLE_MODE )
        setDesiredAngle(val);
    if ( m_controlMode == CM_RATE_MODE )
        setDesiredRate(m_rate*val);
}

void Quad::setDesiredRate(float r)
{
    m_controllingAngle = false;
    m_rateInput = r;
}

void Quad::setDesiredAngle(float a)
{
    m_controllingAngle = true;
    m_angleInput = a;
}
