#include <forecast/controllers/ForcePID_VC_LinMot.hpp>

using namespace forecast;

ForcePID_VC_LinMot::ForcePID_VC_LinMot(float kp, float ki, float kd, float jl, float dl, float kl)
    : kp(kp),
      ki(ki),
      kd(kd),
      Jl(jl),
      Dl(dl),
      Kl(kl),
      errPast(0.f),
      err(0.f),
      derr(0.f),
      ierr(0.f)
{
    logs.push_back(&reference);
    lowPass = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    lowPassD = utility::AnalogFilter::getLowPassFilterHz(40.0f);

    K_motor = 17;

    Jm = 0.76;
    Dm = 0.2;

    //Jl = 7.82;
    //Dl = 0;
    //Kl = 6000;

    N = 1;

    lowPassx = utility::AnalogFilter::getLowPassFilterHz(10.0f);
    lowPassDx = utility::AnalogFilter::getLowPassFilterHz(10.0f);
    lowPassDDx = utility::AnalogFilter::getLowPassFilterHz(10.0f);
}

float ForcePID_VC_LinMot::process(const IHardware *hw, std::vector<float> ref)
{
    //x = lowPassx->process(hw->get_theta(0), hw->get_dt());
    x = hw->get_theta(0);
    //dx = lowPassDx->process(hw->get_d_theta(0), hw->get_dt());
    //ddx = lowPassDDx->process(hw->get_dd_theta(0), hw->get_dt());
    dx = hw->get_d_theta(0);
    ddx = hw->get_dd_theta(0);

    float compensate_1 = ((hw->get_dd_theta(0)*Jm)) + (dx*Dm);


    float compensate_2 = (dx*K_motor*K_e)/Resist;

    float compensate_3 = 0;

    //compensate_2 = 0;


    reference = ref[0];
    //tau = hw->get_tau_s(1);     // was 0: tauS
    //dtau = hw->get_d_tau_s(1);  // was 0: tauS


    tau = lowPass->process(hw->get_tau_s(1), hw->get_dt());
    dtau = lowPassD->process(hw->get_d_tau_s(1), hw->get_dt());

    err = ref[0] - tau;
    derr = (err - errPast) / hw->get_dt();
    ierr += err * hw->get_dt();
    errPast = err;

    *(hw->vel_comp_value) = compensate_2  + compensate_1;

    /*if(dx >= -0.03 && dx <= 0.03 && ((reference - tau) >= 5)){
        compensate_3 = 25;
    }

    else if(dx >= -0.03 && dx <= 0.03 && ((tau - reference) >= 5)){
        compensate_3 = -25;
    }*/

    if(dx > 0.01 && dx < 0.2){
        compensate_3 = 30 - (20/0.2)*dx;
    }

    else if(dx < -0.01 && dx > -0.2){
        compensate_3 = -30 - (20/0.2)*dx;
    }

    compensate_3 = 0;

    out = ref[0] + kp * err + kd * derr + ki * ierr + 1*(compensate_1 + compensate_2 + compensate_3);

    return out;
}