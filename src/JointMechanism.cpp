#include <rosee_gazebo_plugins/JointMechanism.h>

using namespace std;

JointMechanism::JointMechanism(){};

JointMechanism::~JointMechanism(){};

double JointMechanism::Joint1_Func(double J1_DeltAngle_BAC, double Fm)
{

    double J1_k = k1 * pow(0.00747564, 2);              //The stiffness of first joint spring/关节1的刚度 单位： Nm/rad
    double J1_ThetaA = (14.6524 / 180) * pi;            //The upper angle limite of first status/第一状态的上限位 单位： rad
    double J1_ThetaB = (41.765 / 180) * pi;             //The upper angle limite of second status/第二状态的上限为 单位： rad
    double J1_RadiusJoint = 0.003;                      //The radius joint/关节1的直径 单位： m
    double J1_InitAngle_S2_BAC = (154.5895 / 180) * pi; //The initial value when enter status 2/进入第二状态时，角BAC的初始值 单位： rad
    double J1_InitAngle_S3_BAC = (101.899 / 180) * pi;  //The initial value when enter status 3/进入第三状态时，角BAC的初始值 单位： rad
    double J1_Length_AB = 0.0131409;
    double J1_Length_S2_AC = 0.0272947;
    double J1_Length_S3_AC = 0.0126058;

    if (J1_DeltAngle_BAC < J1_ThetaA || J1_DeltAngle_BAC == J1_ThetaA)
    {
        J1_T = Fm * J1_RadiusJoint;
        Func1 = J1_T / J1_k - J1_DeltAngle_BAC;
    }
    else if (J1_DeltAngle_BAC < J1_ThetaB)
    {
        J1_Angle_BAC = J1_InitAngle_S2_BAC - J1_DeltAngle_BAC + J1_ThetaA;
        J1_Length_BC = sqrt((pow(J1_Length_AB, 2) + pow(J1_Length_S2_AC, 2)) - (2 * J1_Length_AB * J1_Length_S2_AC * cos(J1_Angle_BAC)));
        J1_Angle_CBA = asin(J1_Length_S2_AC * sin(J1_Angle_BAC) / J1_Length_BC);
        J1_Angle_CBD = acos(RadiusP / J1_Length_BC);
        J1_Angle_OBD = J1_Angle_CBD - J1_Angle_CBA;
        J1_Length_OB = RadiusP / cos(J1_Angle_OBD);
        J1_Length_AO = J1_Length_AB - J1_Length_OB;
        J1_Deviation = J1_Length_AO * cos(J1_Angle_OBD);
        J1_T = Fm * J1_Deviation;
        Func1 = J1_T / J1_k - J1_DeltAngle_BAC;
    }
    else
    {
        J1_Angle_BAC = J1_InitAngle_S3_BAC - J1_DeltAngle_BAC + J1_ThetaB;
        J1_Length_BC = sqrt((pow(J1_Length_AB, 2) + pow(J1_Length_S3_AC, 2)) - (2 * J1_Length_AB * J1_Length_S3_AC * cos(J1_Angle_BAC)));
        J1_Deviation = J1_Length_S3_AC * J1_Length_AB * sin(J1_Angle_BAC) / J1_Length_BC - RadiusP;
        J1_T = Fm * J1_Deviation;
        Func1 = J1_T / J1_k - J1_DeltAngle_BAC;
    }

    return Func1;
}

double JointMechanism::Joint2_Func(double J2_DeltAngle_BAC, double Fm)
{

    double J2_k = k2 * pow(0.00747564, 2);          //关节2的刚度 单位： Nm/rad
    double J2_InitAngle_BAC = (139.269 / 180) * pi; //关节2的角BAC的初始值 单位： rad
    double J2_Length_AB = 0.0131409;
    double J2_Length_AC = 0.0119219;

    J2_Angle_BAC = J2_InitAngle_BAC - J2_DeltAngle_BAC;
    J2_Length_BC = sqrt((pow(J2_Length_AB, 2) + pow(J2_Length_AC, 2)) - (2 * J2_Length_AB * J2_Length_AC * cos(J2_Angle_BAC)));
    J2_Deviation = J2_Length_AC * J2_Length_AB * sin(J2_Angle_BAC) / J2_Length_BC - RadiusP;
    J2_T = Fm * J2_Deviation;
    Func2 = J2_T / J2_k - J2_DeltAngle_BAC;

    return Func2;
}

double JointMechanism::Joint3_Func(double J3_DeltAngle_BAC, double Fm)
{

    double J3_k = k3 * pow(0.00747564, 2);           //关节3的刚度 单位： Nm/rad
    double J3_InitAngle_BAC = (135.5663 / 180) * pi; //关节3的角BAC的初始值 单位： rad
    double J3_Length_AB = 0.0119219;
    double J3_Length_AC = 0.0144474;

    J3_Angle_BAC = J3_InitAngle_BAC - J3_DeltAngle_BAC;
    J3_Length_BC = sqrt((pow(J3_Length_AB, 2) + pow(J3_Length_AC, 2)) - (2 * J3_Length_AB * J3_Length_AC * cos(J3_Angle_BAC)));
    J3_Angle_CBA = asin(J3_Length_AC * sin(J3_Angle_BAC) / J3_Length_BC);
    J3_Angle_CBD = acos(RadiusP / J3_Length_BC);
    J3_Angle_OBD = J3_Angle_CBD - J3_Angle_CBA;
    J3_Length_OB = RadiusP / cos(J3_Angle_OBD);
    J3_Length_AO = J3_Length_AB - J3_Length_OB;
    J3_Deviation = J3_Length_AO * cos(J3_Angle_OBD);
    J3_T = Fm * J3_Deviation;
    Func3 = J3_T / J3_k - J3_DeltAngle_BAC;

    return Func3;
}
