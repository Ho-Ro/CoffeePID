#pragma once

#include <Arduino.h>

/*
Based on https://github.com/AlexGyver/GyverLibs - PWMrelay
PWMrelay - a library for generating a low-frequency PWM signal for relays (for PID controllers, etc.)
 - Customization of the PWM period
 - Supports low and high level relays
 - Non-blocking call, does not use timers (except for the system one), works on millis ()
*/

class PWMout {
    public:
        PWMout( byte pin, bool dir, unsigned period );	// pin, relay level HIGH / LOW, period
        void tick(); // tick, call as often as possible, controls the relay
        void setPWM( float duty ); // set the PWM fill factor, 0 - 100 (0-100%). With a value of 0 and 100, the tick is inactive!
        float getPWM(); // returns the PWM value
        void setPeriod( unsigned period ); // set the PWM period in milliseconds. (default 1000ms = 1s)
        int getPeriod(); // get period
        void setLevel( bool level ); // set relay level(HIGH / LOW)
        bool isActive(); // current state of relay

    private:
        bool _isActive = false;
        bool _dir = false;
        byte _pin = 0;
        float _duty = 0;
        unsigned _period = 1000;
        unsigned _activePeriod = 0;
        uint32_t _tmr = 0;
        bool _isWorking = false;
};
