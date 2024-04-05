

#include <forecast/controllers/teste.hpp>

using namespace forecast;

#define __sign(x) (std::signbit(x) ? -1.0 : 1.0)

teste::teste(float kp, float ki, float kd
        )
    : kp(kp), 
    ki(ki),
    kd(kd),
{

    lowPass = utility::AnalogFilter::getDifferentiatorHz(20.0f);
}

float teste::process(const IHardware *hw, std::vector<float> ref)
{
    theta = hw->get_theta(1); // hw->getThetaE();

    out=10*theta;

    tau = hw->get_tau_s(1);

    return out;
}
