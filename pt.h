#pragma once

/*
LTI transfer functions PT1 and PT2
used for smoothing of temperature reading and heater simulation
*/



// source: http://webber.physik.uni-freiburg.de/~hon/vorlss02/Literatur/Ingenieurswiss/Regelungstechnik/ProgrammiereRegelung.pdf Seite 9

class PT1 {
    public:
        PT1( double v = 1, double t = 1 ); // v: gain, t: time constant
        double pt1( double input );

    private:
        double a = 0;
        double V;
        double T;
        bool init = true;
};

class PT2 {
    public:
        PT2( double v = 1, double t = 1, double d = 0.7 ); // v: gain, t: time constant, d: damping

        double pt2( double input );

    private:
        double a = 0;
        double b = 0;
        double V;
        double T;
        double D;
        bool init = true;
};
