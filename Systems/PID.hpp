/**********************************************************************************************
* PID Library
* Edited from Arduino PID Library - Version 1.2.1
* by GUrobosub
* Initially written by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
*
* This Library is licensed under the MIT License (maybe?)
**********************************************************************************************/
#include "JetsonLink.hpp"
#include "SystemDriver.hpp"
#include "Definitions.hpp"

#ifndef PID_HPP
#define PID_HPP


namespace Exodus {

class PID : public SystemDriver {
public:
    /**
     * Initialize the PID
     * @params: Value for Constant of Proportional Component (Kp)
     *          Value for Constant of Integral Component (Ki)
     *          Value for Constant of Derivative Component (Kd)
     *          Type of PID input - either Decimal or Angle (inputType)
     *          Pointer to JetsonLink for feedback (tiLink)
     *          Boolean to signify debugging option (debug)
     *          Boolean to dictate when to calculate proportional component (_pOnE)
     * @return: none
     */
    void Initialize(float Kp, float Ki, float Kd,
                    PIDInputType inputType = Decimal, JetsonLink *tilink = 0, bool debug = false, bool _pOnE = true);

    /**
     * Compute the PID output given an input
     * @params: pointer to where to produce output (output)
     *          input value (input)
     * @return: none
     */
    void Compute(float* output, float input);

    /**
     * Reset PID Component values back to 0
     * @params: none
     * @return: none
     */
    void resetPID();

    /**
     * Getters and setters for private data members
     */
    void GetOutputComponents(float* pComponent, float* iComponent, float* dComponent);
    void SetOutputLimits(float OutputMin, float OutputMax);
    void SetDeadband(float DeadbandCenter, float DeadbandLower, float DeadbandHigher);
    void setKp(float Kp);
    void setKi(float Ki);
    void setKd(float Kd);
    float getKp();
    float getKi();
    float getKd();
    void setpOnE(bool pOnE);
    void setSetPoint(float Setpoint);
    float getSetPoint();


private:
    PIDInputType inputType;
    float setPoint;
    float outputSum, lastInput;

    float Kp;  // The (P)roportional Gain Parameter
    float Ki; // The (I)ntegral Gain Parameter
    float Kd; // The (D)erivative Gain Parameter
    bool pOnE; // The boolean to specify if the proportional component is calculated on the error or the measurement
    bool outputLimitsEnabled;

    float outputMax, outputMin;
    float deadbandCenter, deadbandLower, deadbandHigher;
    bool IsDeadbandEnabled; // The deadband is disabled by default

    float pComponent;
    float iComponent;
    float dComponent;

    JetsonLink *TiLink;

    float ApplyDeadband(float value);
    float ComputeDecimal(float input);
    float ComputeAngle(float input);

};


}



#endif // PID_HPP
