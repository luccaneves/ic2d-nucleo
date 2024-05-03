

#include <forecast/controllers/teste.hpp>

using namespace forecast;

#define __sign(x) (std::signbit(x) ? -1.0 : 1.0)

teste::teste(float kp, float ki, float kd
        )
    : kp(kp), 
    ki(ki),
    kd(kd)
{

    lowPass = utility::AnalogFilter::getDifferentiatorHz(20.0f);
}

float teste::process(const IHardware *hw, std::vector<float> ref)
{
    //theta = hw->get_theta(1); // hw->getThetaE();
    //tau = hw->get_tau_s(1);

    Pa = lowPassPa->process(hw->get_pressure(0)*100000,hw->get_dt());
    Pb = lowPassPb->process(hw->get_pressure(1)*100000,hw->get_dt());
    reference = ref[0];

    Pl = Pa - Pb;

    err = ref[0] - Pl; 
    derr = (err - errPast) / hw->get_dt();
    ierr += err * hw->get_dt();

    errPast = err;

    //out=10*theta;


    out = kp * err + ki * ierr + kd * derr;

    return out;
}
