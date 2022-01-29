#include "PWMout.h"

/*
Based on https://github.com/AlexGyver/GyverLibs - PWMrelay
PWMrelay - a library for generating a low-frequency PWM signal for relays (for PID controllers, etc.)
 - Customization of the PWM period
 - Supports low and high level relays
 - Non-blocking call, does not use timers (except for the system one), works on millis ()
*/

PWMout::PWMout( byte pin, bool dir = false, unsigned period = 1000 ) {
    _pin = pin;
    _dir = !dir;
    pinMode( _pin, OUTPUT );
    digitalWrite( _pin, _dir ); // immediately off
    PWMout::setPeriod( period );
}


void PWMout::tick() {
    if ( _isWorking ) {
        if ( millis() - _tmr >= ( _isActive ? _activePeriod : ( _period - _activePeriod ) ) ) {
            _tmr = millis();
            // status has changed
            _isActive = !_isActive;
            digitalWrite( _pin, _isActive ^ _dir );
        }
    }
}


void PWMout::setPWM( float duty ) {
    if ( duty == _duty ) return; // no change
    _duty = duty;
    _activePeriod = ( _duty * _period / 100.0 ) + 0.5;
    _isWorking = false;
    if ( _activePeriod <= 0 ) {
        _activePeriod = 0;
        digitalWrite( _pin, _dir );  // off
        _isActive = false; // for isActive()
    } else if ( _activePeriod >= _period ) {
        _activePeriod = _period;
        digitalWrite( _pin, !_dir ); // on
        _isActive = true; // for isActive()
    } else _isWorking = true;
}


float PWMout::getPWM() {
    return _duty;
}


void PWMout::setPeriod( unsigned period ) {
    _period = period;
    PWMout::setPWM( _duty );	// in case of "hot" period change
}


int PWMout::getPeriod() {
    return _period;
}


void PWMout::setLevel( bool level ) {
    _dir = !level;
}


bool PWMout::isActive() {
    return _isActive;
}
