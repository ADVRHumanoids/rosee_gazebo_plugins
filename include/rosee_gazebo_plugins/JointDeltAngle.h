#ifndef __JOINTDELTANGLE_H_
#define __JOINTDELTANGLE_H_

class JointDeltAngle
{
private:
    double lower_boundary;
    double higher_boundary;
    double Critria;
    double first_step;
    
    JointMechanism JM;

public:
    
    JointDeltAngle();
    ~JointDeltAngle();
    void solver_config(double MinValue, double MaxValue, double Precision, double InitStep);
    double DeltAngle_Join1(double Fm);
    double DeltAngle_Join2(double Fm);
    double DeltAngle_Join3(double Fm);
};

#endif
