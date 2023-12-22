/**********************************************************************************************
* PID Library
* Edited from Arduino PID Library - Version 1.2.1
* by GUrobosub
* Initially written by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
*
* This Library is licensed under the MIT License (maybe?)
**********************************************************************************************/

#include "PID.hpp"
#include "math.h"


void Exodus::PID::Initialize(float Kp, float Ki, float Kd, PIDInputType inputType, JetsonLink *tilink, bool debug, bool pOnE) {
    // Save decided options
    this -> inputType = inputType;

    // Save parameter on proportional on error or measurement
    this -> pOnE = pOnE;

    // Save gain parameters for PID
    this -> Kp = Kp;
    this -> Ki = Ki;
    this -> Kd = Kd;

    // Disable deadband by default
    IsDeadbandEnabled = false;

    // Disable output limits by default
    outputLimitsEnabled = false;

    TiLink = tilink;

    resetPID();
}


void Exodus::PID::SetDeadband(float DeadbandCenter, float DeadbandLower, float DeadbandHigher) {
    deadbandCenter = DeadbandCenter;
    deadbandLower = DeadbandLower;
    deadbandHigher = DeadbandHigher;

    if (deadbandLower < deadbandCenter and deadbandCenter < deadbandHigher) IsDeadbandEnabled = true;
    else IsDeadbandEnabled = false;


}

void Exodus::PID::SetOutputLimits(float OutputMin, float OutputMax) {
    outputLimitsEnabled = true;
    outputMax = OutputMax;
    outputMin = OutputMin;
}

void Exodus::PID::setKp(float Kp) {
    this -> Kp = Kp;
}

void Exodus::PID::setKi(float Ki) {
    this -> Ki = Ki;
}

void Exodus::PID::setKd(float Kd) {
    this -> Kd = Kd;
}

void Exodus::PID::setpOnE(bool pOnE) {
    this -> pOnE = pOnE;
}

float Exodus::PID::getKp() {
    return Kp;
}

float Exodus::PID::getKi() {
    return Ki;
}

float Exodus::PID::getKd() {
    return Kd;
}
void Exodus::PID::setSetPoint(float Setpoint) {
    setPoint = Setpoint;
}

float Exodus::PID::getSetPoint() {
    return setPoint;
}

void Exodus::PID::GetOutputComponents(float *pComponent, float *iComponent, float *dComponent) {
    *pComponent = this -> pComponent;
    *iComponent = this -> iComponent;
    *dComponent = this -> dComponent;
}

void Exodus::PID::resetPID() {
    outputSum = 0;
    iComponent = 0;
    pComponent = 0;
    dComponent = 0;
}

float Exodus::PID::ApplyDeadband(float value) {
    if (!IsDeadbandEnabled) return value;

    // If the input value is between the deadbandCenter and the deadbandHigher value, saturate input value to
    // deadBandHigher
    if (value > deadbandCenter and value < deadbandHigher) value = deadbandHigher;
    if (value < deadbandCenter and value > deadbandLower) value = deadbandLower;

    return value;
}

float Exodus::PID::ComputeDecimal(float input) {
    /* Compute all the working error variables */
    float error = setPoint - input;                       // Calculate error between setpoint and current input
    float dInput = (input - lastInput);                    // Calculate change between current input and last input (derivative)
    outputSum += (Ki * error);                             // Calculate Integral Component

    iComponent = outputSum;                              // Save integral component value

    /* Add Proportional on Measurement, if P_ON_M is specified */
    if (!pOnE) {
        pComponent = Kp * dInput;
        outputSum -= pComponent;                   // Calculate Proportional on Measurement (if specified)
    }

                                                            // Check if within bounds and saturate if not so
    if (outputLimitsEnabled){
        if (outputSum > outputMax)
            outputSum = outputMax;
        else if (outputSum < outputMin)
            outputSum = outputMin;
    }


    /* Add Proportional on Error, if P_ON_M is specified */
    float output;
    if (pOnE)
        output = Kp * error;                          // Calculate Proportional on Error (if specified)
    else
        output = 0;

    /* Compute Rest of PID Output */
    dComponent = Kd * dInput;                             // Save Derivative Component
    output += outputSum - Kd * dInput;                     // Calculate Derivative Component

                                                            // Check if within bounds and saturate if not so
    if (outputLimitsEnabled){
        if (output > outputMax)
            output = outputMax;
        else if (output < outputMin)
            output = outputMin;
    }


                                                            // Save output value
    output = (IsDeadbandEnabled) ? ApplyDeadband(output) : output;

    /* Remember variables for next time*/
    lastInput = input;

    return output;
}

float Exodus::PID::ComputeAngle(float input) {
    /* Compute all the working error variables */
    // _TiLink -> printf("\n%f~", input);
    // Find closest angle (accounting overlap with 360 degrees)
    float error;

    if (fabsf(setPoint - input) > fabsf(setPoint - (input + 360))) {
        // If the setpoint is 360 degrees greater than the input
        error = setPoint - input - 360 ;
    } else if (fabsf(setPoint - input) < fabsf(setPoint - (input - 360))) {
        // If the setpoint is within the same rotation
        error = setPoint - input;
    } else {
        // If the setpoint is 360 less than the input
        error = setPoint - input + 360 ;
    }


    float dInput;

    if (fabsf(lastInput - input) > fabsf(lastInput - (input + 360))) {
        // If the setpoint is 360 degrees greater than the input
        dInput = (input - lastInput + 360);        // maybe +360 instead (altered)
    } else if (fabsf(lastInput - input) < fabsf(lastInput - (input - 360))) {
        // If the setpoint is within the same rotation
        dInput = (input - lastInput);
    } else {
        // If the setpoint is 360 less than the input
        dInput = (input - lastInput - 360);        // maybe -360 instead (altered)
    }


    outputSum += (Ki * error);

    /* Add Proportional on Measurement, if P_ON_M is specified */
    if (!pOnE) outputSum -= Kp * dInput;

    // Check if within bounds and saturate if not so
    if (outputLimitsEnabled) {
        if (outputSum > outputMax) {
            outputSum = outputMax;
        } else if (outputSum < outputMin) {
            outputSum = outputMin;
        }
    }

    /* Add Proportional on Error, if P_ON_M is specified */
    double output;
    if (pOnE)
        output = Kp * error;
    else
        output = 0;

    /* Compute Rest of PID Output */
    output += outputSum - Kd * dInput;

    // Check if within bounds and saturate if not so
    if (outputLimitsEnabled){
        if (output > outputMax) {
            output = outputMax;
        } else if (output < outputMin) {
            output = outputMin;
        }
    }


    // Save output value
    output = (IsDeadbandEnabled) ? ApplyDeadband(output) : output;

    /* Remember variables for next time*/
    lastInput = input;
    return output;
}

void Exodus::PID::Compute(float* output, float input) {
    if (!isRunning()) return;                                 // Leave function if we do not want to run PID

    if (inputType == Decimal)
        *output = ComputeDecimal(input);
    else if (inputType == Angle)
        *output = ComputeAngle(input);

}
