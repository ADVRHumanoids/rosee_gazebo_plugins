#ifndef _JOINTMECHANISM_H_
#define _JOINTMECHANISM_H_

#include <iostream>
#include <math.h>

/**
 * Credits to @EvanZhangYifang
 */

class JointMechanism
{
private:
    const double pi = 3.141592653589793;
    const double RadiusP = 0.0013;
    double k1 = 880;
    double k2 = 950;
    double k3 = 1660;
    double J1_T,
        J1_Angle_BAC,
        J1_Length_BC,
        J1_Angle_CBA,
        J1_Angle_CBD,
        J1_Angle_OBD,
        J1_Length_OB,
        J1_Length_AO,
        J1_Deviation;
    double J2_T,
        J2_Angle_BAC,
        J2_Length_BC,
        J2_Deviation;
    double J3_T,
        J3_Angle_BAC,
        J3_Length_BC,
        J3_Angle_CBA,
        J3_Angle_CBD,
        J3_Angle_OBD,
        J3_Length_OB,
        J3_Length_AO,
        J3_Deviation;

public:
    double Func1;
    double Func2;
    double Func3;
    JointMechanism();
    ~JointMechanism();
    double Joint1_Func(double J1_DeltAngle_BAC, double Fm);
    double Joint2_Func(double J2_DeltAngle_BAC, double Fm);
    double Joint3_Func(double J3_DeltAngle_BAC, double Fm);
};

#endif
