#include <forecast/controllers/ForcePID.hpp>

using namespace forecast;

ForcePID::ForcePID(float kp, float ki, float kd)
    : kp(kp),
      ki(ki),
      kd(kd),
      errPast(0.f),
      err(0.f),
      derr(0.f),
      ierr(0.f)
{
    logs.push_back(&reference);
    lowPass = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    lowPassD = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    lowPassPa = utility::AnalogFilter::getLowPassFilterHz(10.0f);
    lowPassPb = utility::AnalogFilter::getLowPassFilterHz(10.0f);
}

float ForcePID::process(const IHardware *hw, std::vector<float> ref)
{
    reference = ref[0];
    //tau = hw->get_tau_s(1);     // was 0: tauS
    //dtau = hw->get_d_tau_s(1);  // was 0: tauS

    //tau = lowPass->process(hw->get_tau_s(1), hw->get_dt());
    //dtau = lowPassD->process(hw->get_d_tau_s(1), hw->get_dt());

    float Pa =  lowPassPa->process(hw->get_pressure(2)*100000,hw->get_dt());
    float Pb = lowPassPb->process(hw->get_pressure(3)*100000,hw->get_dt());
 
    Pl = Pa - Pb*0.609375;

    tau = hw->get_tau_s(0);
    dtau = hw->get_d_tau_s(0);

    if(hw->get_current_time() > 5 && once_rise_time_flag == 0 && tau > ref[0]*0.1){
            once_rise_time_flag = 1;
            rise_time_start = hw->get_current_time();
    }

    else if(once_2_rise_time_flag == 0 && once_rise_time_flag == 1 && hw->get_current_time() > 5 && tau > ref[0]*0.9){
            rise_time_end = hw->get_current_time();
            once_2_rise_time_flag = 1;
    }
    
    if(tau > Mv && hw->get_current_time() > 5){
        Mv = tau;
    }

    err = ref[0] - tau;
    //Lucca TO DO: Corrigir essa derivada. Fazer inf dif com mais pontos
    derr = (err - errPast) / hw->get_dt();
    ierr += err * hw->get_dt();
    errPast = err;

     //Lucca TO DO: Comentar ref[0] depois

    out = kp * err + kd * derr + ki * ierr;

    *(hw->var1) = out;
    *(hw->var2) = rise_time_end - rise_time_start;
    *(hw->var6) = Mv;
    *(hw->var7) = Pl;
    *(hw->var9) = ref[0]; //Forca desejada

    return out;
}