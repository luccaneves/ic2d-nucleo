

#include <forecast/controllers/teste.hpp>

using namespace forecast;

#define __sign(x) (std::signbit(x) ? -1.0 : 1.0)

teste::teste(float kp, float ki, float kd, float fix_leak
        )
    : kp(kp), 
    ki(ki),
    kd(kd),
    fix_leak(fix_leak)
{

    lowPass = utility::AnalogFilter::getDifferentiatorHz(10.0f);
    lowPassPa = utility::AnalogFilter::getDifferentiatorHz(10.0f);
    lowPassPb = utility::AnalogFilter::getDifferentiatorHz(10.0f);

    logs.push_back(&reference);
}

float teste::process(const IHardware *hw, std::vector<float> ref)
{
    //theta = hw->get_theta(1); // hw->getThetaE();
    //tau = hw->get_tau_s(1);

    Pa = hw->get_pressure(0);
    Pb = hw->get_pressure(1);
    reference = ref[0];

    Pl = Pa - Pb*0.75;

    err = ref[0] - Pl; 
    derr = (err - errPast) / hw->get_dt();
    ierr += err * hw->get_dt();

    errPast = err;
    

    //out=10*theta;


    out = kp * err + ki * ierr + kd * derr;


    *(hw->fric1) = Pa;
    *(hw->fric2) = Pb;
    *(hw->control_signal_teste) = out + fix_leak;
    *(hw->sprint_start_force) = Pl;

    return out + fix_leak;
}
