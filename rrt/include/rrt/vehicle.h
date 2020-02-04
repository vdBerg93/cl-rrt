#ifndef VEHICLE_H
#define VEHICLE_H
#include <vector>

class Vehicle{
    public:
        double dmax;    // Maximum steering angle
        double ddmax;   // Maximum steering rate
        double Td;      // Steer damping
        double Ta;      // Acceleration damping
        double amin;    // Minimum acceleration
        double amax;    // Maximum acceleration
        double L;       // Wheel base
        double w;       // Chassis width
        double Lrear;   // Chassis length behind rear axle
        double Lfront;  // Chassis length in front of rear axle
        double b;       // COG positition w.r.t. rear axle
        double Vch;     // Characterisic velocity
        double rho;     // Minimum turning radius
        double Kus;     // Understeer gradient

        void setTalos(){
            dmax = 0.5435;          // From article      
            ddmax = 0.3294;         // From article
            Td = 0.3;               // From article
            Ta = 0.3;               // From article
            amin = -6;              // From article
            amax = 2;               // From article
            L = 2.885;              // From article
            Lrear = 1;              // From image measurement
            Lfront = 4.848-Lrear;   // From image measurement
            w = 2;                  // From article
            b = 1.8;                // Wild guess
            Vch = 20;               // From article
            rho = 4.77;             // From article
            Kus = 0.018;            // Manually tuned
        };

        void setPrius(){
            dmax = 0.52;            // From specs     
            ddmax = 0.3294;         // From article
            Td = 0.3;               // From article
            Ta = 0.3;               // From article
            amin = -6;              // From article
            amax = 2;               // From article
            L = 2.7;                // From specs
            double lf = 1.0868;     // From specs
            double lr = 1.6132;     // From specs
            Lrear = 1;              // Guessed, from rear axle to rear bumper
            Lfront = 2.7+0.5;       // Guessed, from rear axle to front bumper
            w = 2;               // Internet: 1.76m -> round up 2m
            b = lr;                 // From specs
            rho = 5.95;             // From specs

            double Cf = 22201;      // From specs
            double Cr = 22201;      // From specs
            double m = 950+640;     // From specs
            Kus = (m/L)*(lr/Cf - lf/Cr);
            Vch = 20;
        }
};

#endif