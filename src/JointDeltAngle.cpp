#include <rosee_gazebo_plugins/JointDeltAngle.h>

JointDeltAngle::JointDeltAngle()
{
    lower_boundary = -1;
    higher_boundary = 1;
    Critria = 0.000001;
    first_step = 1;
}

JointDeltAngle::~JointDeltAngle(){};

void JointDeltAngle::solver_config(double MinValue, double MaxValue, double Precision, double InitStep)
{
    lower_boundary = MinValue;
    higher_boundary = MaxValue;
    Critria = Precision;
    first_step = InitStep;
}

double JointDeltAngle::DeltAngle_Join1(double Fm)
{
    double a = lower_boundary;
    double b = higher_boundary;
    double f = Fm;
    double yc = 1;
    double c, ya, yb, J1_DeltAngle_BAC;

    do
    {
        ya = JM.Joint1_Func(a, f);
        yb = JM.Joint1_Func(b, f);
        if (ya == 0 || yb == 0) //判断是否正好为根
        {
            if (ya == 0)
            {
                J1_DeltAngle_BAC = a;
                break;
            }
            else
            {
                J1_DeltAngle_BAC = b;
                break;
            }
        }
        else
        {
            if (ya * yb > 0) //当值均在函数根的同一侧
            {
                b = b + first_step;
            }
            else
            {
                c = (a + b) / 2;
                yc = JM.Joint1_Func(c, f);
                if (yc == 0)
                {
                    J1_DeltAngle_BAC = c;
                    break;
                }
                else if (ya * yc < 0)
                {
                    b = c;
                    yb = yc;
                    J1_DeltAngle_BAC = (a + b) / 2;
                }
                else
                {
                    a = c;
                    ya = yc;
                    J1_DeltAngle_BAC = (a + b) / 2;
                }
            }
        }
    } while (fabs(ya - yb) > Critria && yc != 0);

    return J1_DeltAngle_BAC;
}

double JointDeltAngle::DeltAngle_Join2(double Fm)
{
    double a = lower_boundary;
    double b = higher_boundary;
    double yc = 1;
    double f = Fm;
    double c, ya, yb, J2_DeltAngle_BAC;

    do
    {
        ya = JM.Joint2_Func(a, f);
        yb = JM.Joint2_Func(b, f);
        if (ya == 0 || yb == 0) //判断是否正好为根
        {
            if (ya == 0)
            {
                J2_DeltAngle_BAC = a;
                break;
            }
            else
            {
                J2_DeltAngle_BAC = b;
                break;
            }
        }
        else
        {
            if (ya * yb > 0) //当值均在函数根的同一侧
            {
                b = b + first_step;
            }
            else
            {
                c = (a + b) / 2;
                yc = JM.Joint2_Func(c, f);
                if (yc == 0)
                {
                    J2_DeltAngle_BAC = c;
                    break;
                }
                else if (ya * yc < 0)
                {
                    b = c;
                    yb = yc;
                    J2_DeltAngle_BAC = (a + b) / 2;
                }
                else
                {
                    a = c;
                    ya = yc;
                    J2_DeltAngle_BAC = (a + b) / 2;
                }
            }
        }
    } while (fabs(ya - yb) > Critria && yc != 0);
    return J2_DeltAngle_BAC;
}

double JointDeltAngle::DeltAngle_Join3(double Fm)
{
    double a = lower_boundary;
    double b = higher_boundary;
    double yc = 1;
    double f = Fm;
    double c, ya, yb, J3_DeltAngle_BAC;

    do
    {
        ya = JM.Joint3_Func(a, f);
        yb = JM.Joint3_Func(b, f);
        if (ya == 0 || yb == 0) //判断是否正好为根
        {
            if (ya == 0)
            {
                J3_DeltAngle_BAC = a;
                break;
            }
            else
            {
                J3_DeltAngle_BAC = b;
                break;
            }
        }
        else
        {
            if (ya * yb > 0) //当值均在函数根的同一侧
            {
                b = b + first_step;
            }
            else
            {
                c = (a + b) / 2;
                yc = JM.Joint3_Func(c, f);
                if (yc == 0)
                {
                    J3_DeltAngle_BAC = c;
                    break;
                }
                else if (ya * yc < 0)
                {
                    b = c;
                    yb = yc;
                    J3_DeltAngle_BAC = (a + b) / 2;
                }
                else
                {
                    a = c;
                    ya = yc;
                    J3_DeltAngle_BAC = (a + b) / 2;
                }
            }
        }
    } while (fabs(ya - yb) > Critria && yc != 0);
    return J3_DeltAngle_BAC;
}
