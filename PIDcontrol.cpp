#include "PIDcontrol.h"

// Based on Arduino "AutoPID" library https://github.com/r-downing/AutoPID
// - added https://github.com/r-downing/AutoPID/issues/11 (Derivative error time factor equation)
// - changed "void PIDcontrol::run()" to "bool PIDcontrol::run()", returns true if calculation done
// - added getError() and getDError() to debug/tune the PID


PIDcontrol::PIDcontrol( double *input, double *setpoint, double *output, double outputMin, double outputMax,
                  double Kp, double Ki, double Kd ) {
    _input = input;
    _setpoint = setpoint;
    _output = output;
    _outputMin = outputMin;
    _outputMax = outputMax;
    setGains( Kp, Ki, Kd );
    _timeStep = 1000;
}//PIDcontrol::PIDcontrol

void PIDcontrol::setGains( double Kp, double Ki, double Kd ) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
}//PIDcontrol::setControllerParams

void PIDcontrol::setBangBang( double bangOn, double bangOff ) {
    _bangOn = bangOn;
    _bangOff = bangOff;
}//void PIDcontrol::setBangBang

void PIDcontrol::setBangBang( double bangRange ) {
    setBangBang( bangRange, bangRange );
}//void PIDcontrol::setBangBang

void PIDcontrol::setOutputRange( double outputMin, double outputMax ) {
    _outputMin = outputMin;
    _outputMax = outputMax;
}//void PIDcontrol::setOutputRange

void PIDcontrol::setTimeStep( unsigned long timeStep ) {
    _timeStep = timeStep;
}//void PIDcontrol::setTimeStep

bool PIDcontrol::atSetPoint( double threshold ) {
    return abs( *_setpoint - *_input ) <= threshold;
}//bool PIDcontrol::atSetPoint

bool PIDcontrol::run() {
    _newOutput = false; // new output available
    if ( _stopped ) {
        _stopped = false;
        reset();
    }
    //if bang thresholds are defined and we're outside of them, use bang-bang control
    if ( _bangOn && ( ( *_setpoint - *_input ) > _bangOn ) ) {
        *_output = _outputMax;
        reset();
    } else if ( _bangOff && ( ( *_input - *_setpoint ) > _bangOff ) ) {
        *_output = _outputMin;
        reset();
    } else {  //otherwise use PID control
        unsigned long _dT = millis() - _lastStep;  //calculate time since last update
        if ( _dT >= _timeStep ) {                  //if long enough, do PID calculations
            _lastStep = millis();
            _error = *_setpoint - *_input;
            _integral += ( _error + _previousError ) / 2 * _dT / 1000.0; //Riemann sum integral
            _integral = constrain( _integral, _outputMin / _Ki, _outputMax / _Ki ); // prevent wind-up
            _dError = ( _error - _previousError ) / _dT * 1000.0; //derivative
            _previousError = _error;
            double PID = ( _Kp * _error ) + ( _Ki * _integral ) + ( _Kd * _dError );
            *_output = constrain( PID, _outputMin, _outputMax );
            _newOutput = true;
        }
    }
    return _newOutput;
}//bool PIDcontrol::run

void PIDcontrol::stop() {
    _stopped = true;
    reset();
}//void PIDcontrol::stop

void PIDcontrol::reset() {
    _lastStep = millis();
    _integral = 0;
    _error = 0;
    _previousError = 0;
    _newOutput = true;
}//void PIDcontrol::reset

bool PIDcontrol::isStopped() {
    return _stopped;
}//bool PIDcontrol::isStopped

double PIDcontrol::getIntegral() {
    return _integral;
}//double PIDcontrol::getIntegral

double PIDcontrol::getError() {
    return _error;
}//double PIDcontrol::getError

double PIDcontrol::getDError() {
    return _dError;
}//double PIDcontrol::getDError

void PIDcontrol::setIntegral( double integral ) {
    _integral = integral;
}//void PIDcontrol::setIntegral
