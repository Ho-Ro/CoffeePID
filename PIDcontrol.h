#pragma once

#include <Arduino.h>

// Based on Arduino "AutoPID" library https://github.com/r-downing/AutoPID
// - added #11 (Derivative error time factor equation)
// - changed "void PIDcontrol::run()" to "bool PIDcontrol::run()", returns true when new calculation done

class PIDcontrol {

  public:
    // Constructor - takes pointer inputs for control variales, so they are updated automatically
    PIDcontrol(double *input, double *setpoint, double *output, double outputMin, double outputMax,
            double Kp, double Ki, double Kd);

    // Allows manual adjustment of gains
    void setGains(double Kp, double Ki, double Kd);
    
    // Sets bang-bang control ranges, separate upper and lower offsets, zero for off
    void setBangBang(double bangOn, double bangOff);
    
    // Sets bang-bang control range +-single offset
    void setBangBang(double bangRange);
    
    // Allows manual readjustment of output range
    void setOutputRange(double outputMin, double outputMax);
    
    // Allows manual adjustment of time step (default 1000ms)
    void setTimeStep(unsigned long timeStep);
    
    // Returns true when at set point (+-threshold)
    bool atSetPoint(double threshold);
    
    // Runs PID calculations when needed. Should be called repeatedly in loop.
    // Automatically reads input and target and sets output via pointers
    // returns true when new calculation was done
    bool run(); 
    
    // Stops PID functionality, output sets to 
    void stop();
    
    void reset();
    
    bool isStopped();

    double getIntegral();
    double getError();
    double getDError();
    
    void setIntegral(double integral);

  private:
    double _Kp, _Ki, _Kd;
    double _integral, _error, _dError, _previousError;
    double _bangOn, _bangOff;
    double *_input, *_setpoint, *_output;
    double _outputMin, _outputMax;
    unsigned long _timeStep, _lastStep;
    bool _stopped, _newOutput;

};//class PIDcontrol
