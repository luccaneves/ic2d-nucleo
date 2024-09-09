#include <forecast/controllers/ForcePID_DOB.hpp>

using namespace forecast;

ForcePID_DOB::ForcePID_DOB(float kp, float ki, float kd,float GainDOB, float GainVC,float Jm, float Dm, float Ke)
    : kp(kp),
      ki(ki),
      kd(kd),
      Jm(Jm),
      Dm(Dm),
      K_e(Ke),
      GainDOB(GainDOB),
      GainVC(GainVC),
      errPast(0.f),
      err(0.f),
      derr(0.f),
      ierr(0.f)
{
    float freq_corte = 15;//Hz
    logs.push_back(&reference);
    lowPass = utility::AnalogFilter::getLowPassFilterHz(freq_corte);
    lowPassD = utility::AnalogFilter::getLowPassFilterHz(freq_corte);
    lowPassDForce = utility::AnalogFilter::getLowPassFilterHz(freq_corte);
    lowPassControl = utility::AnalogFilter::getLowPassFilterHz(freq_corte);

    K_motor = 17;

    //Jm = 8.76;
    //Dm = 150.1;
}



float ForcePID_DOB::process(const IHardware *hw, std::vector<float> ref)
{
    float x = hw->get_theta(0);
    float dx = hw->get_d_theta(0);

    float compensate_1 = ((hw->get_dd_theta(0)*Jm)) + (dx*Dm);

    float compensate_2 = 0;

    float compensate_3 = 0;

    *(hw->vel_comp_value) = GainVC*( compensate_2  + compensate_1);

    double inv_model_num[6] = { 0   ,1.592230835850342 , -1.582310443952793 ,0,0 , 0};

    double inv_model_den[6] = {1.000000000000000  ,-1.984087638487695  , 0.984127320055285, 0   ,0,0};


    double filter_num[6] = {0  , 0.310425427160331E-4  , 0.308362809159582E-4 ,0,0,0};
    double filter_den[6] = {1.000000000000000  ,-1.980136794483123 ,  0.980198673306755,0,0,0};
    //double filter_den[6] = {0, 0,0,0,0,0};

    double inv_model_exit = 0;

    double filter_exit = 0;

    if(hw->get_current_time() > 0){

        float value = hw->get_tau_s(1);
        value = dx;

        inv_model_exit = 
        inv_model_num[0]*value + 
        inv_model_num[1]*controller_prev1_tauSensor + 
        inv_model_num[2]*controller_prev2_tauSensor +
        inv_model_num[3]*controller_prev3_tauSensor +
        inv_model_num[4]*controller_prev4_tauSensor +
        inv_model_num[5]*controller_prev5_tauSensor 
        -
        inv_model_den[1]*prev1_inv_model_exit - 
        inv_model_den[2]*prev2_inv_model_exit -
        inv_model_den[3]*prev3_inv_model_exit -
        inv_model_den[4]*prev4_inv_model_exit -
        inv_model_den[5]*prev5_inv_model_exit;

        inv_model_exit = inv_model_exit/inv_model_den[0];

        filter_exit = 
        filter_num[0]*(*(hw->fric2)) + 
        filter_num[1]*controller_prev1_tauM + 
        filter_num[2]*controller_prev2_tauM + 
        filter_num[3]*controller_prev3_tauM + 
        filter_num[4]*controller_prev4_tauM  
        -
        filter_den[1]*prev1_filter_exit - 
        filter_den[2]*prev2_filter_exit -
        filter_den[3]*prev3_filter_exit -
        filter_den[4]*prev4_filter_exit -
        filter_den[5]*prev5_filter_exit;

        filter_exit = filter_exit/filter_den[0];



        prev5_filter_exit = prev4_filter_exit;
        prev4_filter_exit = prev3_filter_exit;
        prev3_filter_exit = prev2_filter_exit;
        prev2_filter_exit = prev1_filter_exit;
        prev1_filter_exit = filter_exit;

        prev5_inv_model_exit = prev4_inv_model_exit;
        prev4_inv_model_exit = prev3_inv_model_exit;
        prev3_inv_model_exit = prev2_inv_model_exit;
        prev2_inv_model_exit = prev1_inv_model_exit;
        prev1_inv_model_exit = inv_model_exit;

        controller_prev6_tauM = controller_prev5_tauM;
        controller_prev5_tauM = controller_prev4_tauM;
        controller_prev4_tauM = controller_prev3_tauM;
        controller_prev3_tauM = controller_prev2_tauM;
        controller_prev2_tauM = controller_prev1_tauM;
        controller_prev1_tauM = *(hw->fric2);

        controller_prev6_tauSensor = controller_prev5_tauSensor;
        controller_prev5_tauSensor = controller_prev4_tauSensor;
        controller_prev4_tauSensor = controller_prev3_tauSensor;
        controller_prev3_tauSensor = controller_prev2_tauSensor;
        controller_prev2_tauSensor = controller_prev1_tauSensor;
        controller_prev1_tauSensor = value;
    }



    reference = ref[0];
    //tau = hw->get_tau_s(1);     // was 0: tauS
    //dtau = hw->get_d_tau_s(1);  // was 0: tauS

    //tau = lowPass->process(hw->get_tau_s(1), hw->get_dt());
    //dtau = lowPassD->process(hw->get_d_tau_s(1), hw->get_dt());

    tau = hw->get_tau_s(1);
    dtau = hw->get_d_tau_s(1);

    err = ref[0] - tau;
    //derr = (err - errPast) / hw->get_dt();

    float derr_no_filt = (2.45*err - 6*prev1_err + 7.5*prev2_err - 6.66*prev3_err 
    + 3.75*prev4_err - 1.2*prev5_err + 0.16*prev6_err)/
    (hw->get_dt());


    derr = lowPassDForce->process(derr_no_filt, hw->get_dt());

    derr = derr_no_filt;

    ierr += err * hw->get_dt();
    errPast = err;

    //*(hw->tauM) = filter_exit;

    out = ref[0] + kp * err + kd * derr + ki * ierr;

    double dob_exit = (tau + inv_model_exit) - filter_exit;
    //double dob_exit = (inv_model_exit) - filter_exit;

    int a = 45;

    if(dob_exit > a){
        dob_exit = a;
    }
    if(dob_exit < -a){
        dob_exit = -a;
    }


    if(tau > max_tau && hw->get_current_time() > 2){
        max_tau = tau;
    }

    if(once_start == 1 && tau > 0.1*ref[0] && hw->get_current_time() > 2){
        once_start = 0;
        time_start = hw->get_current_time();
    }
    else if(once_end == 1 && tau > 0.9*ref[0] && once_start == 0 && hw->get_current_time() > 2){
        once_end = 0;
        time_end= hw->get_current_time();
    }
    else if(once_end == 0 && once_start == 0 && hw->get_current_time() > 2){
        rise_time = time_end - time_start;
    }


    *(hw->fric1) = rise_time;
    /**(hw->fric2) = max_tau;*/

    prev6_err = prev5_err;
    prev5_err = prev4_err;
    prev4_err = prev3_err;
    prev3_err = prev2_err;
    prev2_err = prev1_err;
    prev1_err = err;

    float control_output_no_filter = (out - GainDOB*dob_exit + GainVC*(compensate_2  + compensate_1));

    float out_control = lowPassControl->process(control_output_no_filter, hw->get_dt());

    *(hw->fric2) = control_output_no_filter;

    return (control_output_no_filter);
}