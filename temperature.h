#pragma once

// returns temperature in °C (0 = error)
double getTemperature();

// simulate the boiler increase/decrease over time
double simulateTemperature( bool heat );
