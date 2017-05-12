#ifndef PIDController_H
#define PIDController_H

/*
    Simple PID controller for single float variable
    http://en.wikipedia.org/wiki/PID_controller#Pseudocode
*/

class PIDController {
protected:
    float m_gainP;
    float m_gainI;
    float m_gainD;

    float m_setPoint;
    float m_currentPoint;
    float m_previousPoint;

    float m_integral;
    float m_output;

public:
    PIDController() {
        m_currentPoint = m_previousPoint = m_integral = m_output = 0;
        m_gainP = m_gainP = m_gainP = 1;
    }

    void setGains(float p, float i, float d) { m_gainP = p; m_gainI = i; m_gainD = d; }

    void setSetPointAndCurrentPoint( float s, float c ) { m_setPoint = s; m_currentPoint = c; }

    void step(float dt) {
        float error = m_setPoint - m_currentPoint;
        //m_integral = dt * (m_integral + error);
        m_integral = m_integral + dt * error;
        float derivativeOnMeasurement = - (1/dt) * (m_currentPoint - m_previousPoint);
        m_output = m_gainP * error + m_gainI * m_integral + m_gainD * derivativeOnMeasurement;
        m_previousPoint = m_currentPoint;
    }

    void reset() {
        m_currentPoint = m_previousPoint = m_integral = m_output = 0;
    }

    float setP(float p) { m_gainP = p; }
    float setI(float i) { if ( m_gainI != 0 ) m_integral *= i/m_gainI; m_gainI = i;  }
    float setD(float d) { m_gainD = d; }
    float getP() { return m_gainP; }
    float getI() { return m_gainI; }
    float getD() { return m_gainD; }

    float getOutput() { return m_output; }
};

#endif
