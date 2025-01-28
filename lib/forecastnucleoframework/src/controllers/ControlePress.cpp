

#include <forecast/controllers/ControlePress.hpp>

using namespace forecast;

#define __sign(x) (std::signbit(x) ? -1.0 : 1.0)

ControlePress::ControlePress(float kp, float ki, float kd, float fix_leak
        )
    : kp(kp), 
    ki(ki),
    kd(kd),
    fix_leak(fix_leak)
{

    lowPass = utility::AnalogFilter::getLowPassFilterHz(10.0f);
    lowPassPa = utility::AnalogFilter::getLowPassFilterHz(5.0f);
    lowPassPb = utility::AnalogFilter::getLowPassFilterHz(5.0f);

    logs.push_back(&reference);
}

float ControlePress::process(const IHardware *hw, std::vector<float> ref)
{
    //theta = hw->get_theta(1); // hw->getThetaE();
    //tau = hw->get_tau_s(1);
    dx = hw->get_d_theta(1);

    Pa =  lowPassPa->process(hw->get_pressure(2)*100000,hw->get_dt());
    Pb = lowPassPb->process(hw->get_pressure(3)*100000,hw->get_dt());
    reference = ref[0]*100000; //Em bar no aplicativo

    Pl = Pa - Pb*0.609375;

    float Pl_filter = (last_Pl_1*0.016528546178383 + 0.983471453821617*last_Pl_filter_1);

    last_Pl_1 = Pl;
    last_Pl_filter_1 = Pl_filter;

    //dPl = (Pl - PlPast) / hw->get_dt();

    //PlPast = Pl;

    //Fb = 700*dx; //Viscous friction

    err = reference + ((700*dx)/(0.00020106)) - Pl_filter; 
    derr = (err - errPast) / hw->get_dt();
    ierr += err * hw->get_dt();

    errPast = err;

    //out=10*theta;


    out = (kp/1000000) * err + (ki/1000000) * ierr + (kd/1000000) * derr;

    //tau = hw->get_tau_s(0);
    //dtau = hw->get_d_tau_s(0);

    if(hw->get_current_time() > 5 && once_rise_time_flag == 0 && Pl > reference*0.1){
            once_rise_time_flag = 1;
            rise_time_start = hw->get_current_time();
    }

    else if(once_2_rise_time_flag == 0 && once_rise_time_flag == 1 && hw->get_current_time() > 5 && Pl > reference*0.9){
            rise_time_end = hw->get_current_time();
            once_2_rise_time_flag = 1;
    }

    if(Pl > Mv && hw->get_current_time() > 5){
        Mv = Pl;
    }

    *(hw->var1) = out;
    *(hw->var2) = rise_time_end - rise_time_start;
    *(hw->var6) = Mv/100000;
    *(hw->var7) = Pl/100000;
    *(hw->var9) = ref[0]; //Pl desejado

    return out + fix_leak;
}
