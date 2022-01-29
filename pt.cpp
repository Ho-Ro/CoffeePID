/*
LTI functions PT1 and PT2
used for smoothing of temperature reading and heater simulation
*/

#include "pt.h"


// source: http://webber.physik.uni-freiburg.de/~hon/vorlss02/Literatur/Ingenieurswiss/Regelungstechnik/ProgrammiereRegelung.pdf Seite 9


PT1::PT1( double v, double t ) : V( v ), T ( t ) {};

double PT1::pt1( double input ) {
    if ( init ) {
        a = input;
        init = false;
    }
    double da = T * ( V * input - a );
    a += da;
    return a;
}


PT2::PT2( double v, double t, double d ) : V( v ), T ( t ), D( d ) {};

double PT2::pt2( double input ) {
    if ( init ) {
        a = input;
        init = false;
    }
    double da = T * b;
    double db = T * ( V * input - a - 2 * D * b );
    a += da;
    b += db;
    return a;
}
