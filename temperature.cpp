#include <Arduino.h>
#include "pt.h"

double getTemperature() {
    //
    //  measure the voltage of the divider and calculate the temperature
    //
    //  Vdd 3.3V
    //   |
    //  I I
    //  I I  R1 (1000 Ohm)
    //  I I
    //   +--------> A0 (uNTC)
    //  I I
    //  I I  NTC (645@100°C / B=4000)
    //  I I
    //   |
    //  GND
    //
    //  calculate around 100°C
    //  R_NTC@100°C ~ 645 Ohm -> R1 = 1k to be most sensitive in the range 80 .. 110 °C
    //
    const uint8_t TEMP_PIN = A0;
    const double VDD = 3.3;           // Supply Voltage (adjust for your ESP)
    const double VMAX = 3.07;         // @A0 -> AnalogRead = 1024 (adjust for your ESP)
    const double R1 = 1000;
    const double BETA = 4000;         // NTC beta parameter (adjust for your sensor)
    const double R100C = 645;         // NTC resistance @ 100°C (adjust for your sensor)
    const double KCOff = 273.15;      // 0° Celsius in Kelvin
    const double T100C = 100 + KCOff; // NTC nominal temperature (100°C) in Kelvin

    double rNTC;       // calculated NTC resistance
    double tK;         // calculated NTC temperature in K
    static double tC;  // calculated NTC temperature in °C

    const unsigned repeat = 10;

    static unsigned long lastRun = millis();
    unsigned long now = millis();
    if ( now - lastRun >= repeat ) { // time reached
        lastRun = now;

        unsigned raw = analogRead( TEMP_PIN );
        if ( raw < 20 || raw > 1000 ) // short or open sensor
            return 0; // error
            
        double valueNTC = double( raw ) / 1024; // range 0..1024 on ESP

        // calculate ntc resistance
        rNTC = ( ( valueNTC ) / ( 1 - valueNTC ) ) * R1 * VMAX / VDD;
        // calculate temperature in Kelvin
        tK = 1 / ( ( 1 / T100C ) + ( 1 / BETA ) * log( rNTC / R100C ) );
        // calculate temperature in °C
        tC = tK - KCOff;

        if ( tC <= 0 )
            return 0; // error

        //Serial.print( tC );
        //Serial.print( " " );

        // apply a PT1 low pass filter
        static PT1 myPT1( 1.0, 0.01 );
        tC = myPT1.pt1( tC ); // filter noise

        //Serial.println( tC );
    }
    return tC;
}



// simulation of heat up / cool down / cold water flow
double simulateTemperature( bool heat ) {
    const double tGradHeat = 0.5 / 1000; // K/ms
    const double tGradCool = 0.05 / 1000; // K/ms
    static unsigned disturb = 0; // from time to time the temp drops
    static double heater = 80; // starting temp
    const unsigned repeat = 20;

    static double temperature = heater;

    static PT2 myPT2( 1, 0.005, 0.7 ); // slow rise with some overshoot

    static unsigned long lastRun = millis();
    unsigned long now = millis();
    if ( now - lastRun >= repeat ) { // time reached
        lastRun = now;

        if ( heat )
            heater += tGradHeat * repeat;
        else
            heater -= tGradCool * repeat * heater / 100; // cool down depending on boiler temp

        if ( 0 == ++disturb % 6000 ) // disturb every 60 s with some random value (also up)
            heater += 2 - random( 10 );

        heater = constrain( heater, 20, 140 ); // limit between room temp and steam temp
        temperature = myPT2.pt2( heater ); // smooth function

#if 0
        // debug output for serial plotter
        Serial.print( heat ? 100 : 96 );
        Serial.print( " " );
        Serial.print( heater );
        Serial.print( " " );
        Serial.println( temperature );
#endif
    }
    return temperature;
}
