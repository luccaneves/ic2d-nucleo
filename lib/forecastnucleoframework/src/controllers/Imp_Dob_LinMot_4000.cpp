#include <forecast/controllers/Imp_Dob_LinMot_4000.hpp>

using namespace forecast;

Imp_Dob_LinMot_4000::Imp_Dob_LinMot_4000(float kp, float ki, float kd,float Ides, float Ddes, float Kdes, float DobGain, float Jl, float Bl, float Kl)
    : kp(kp),
      ki(ki),
      kd(kd),
      Kdes(Kdes),
      Ddes(Ddes),
      Ides(Ides),
      Jl(Jl),
      Bl(Bl),
      Kl(Kl),
      DobGain(DobGain),
      errPast(0.f),
      err(0.f),
      derr(0.f),
      ierr(0.f)
{
    logs.push_back(&reference);
    lowPass = utility::AnalogFilter::getLowPassFilterHz(10.0f);
    lowPassD = utility::AnalogFilter::getLowPassFilterHz(10.0f);
    lowPassDD = utility::AnalogFilter::getLowPassFilterHz(10.0f);
}



float Imp_Dob_LinMot_4000::process(const IHardware *hw, std::vector<float> ref)
{
    tau = hw->get_tau_s(1);
    dtau = hw->get_d_tau_s(1);
    theta = hw->get_theta(0);
    dtheta = hw->get_d_theta(0);
    ddtheta = hw->get_dd_theta(0);
    dtheta_filt = lowPassD->process(dtheta, hw->get_dt());
    ddtheta_filt = lowPassDD->process(ddtheta, hw->get_dt());

    /* Get the equilibrium state */
    if (once)
    {
        theta_eq = theta;
        once = false;
        errPast = 0;
    }

    /* POSITION LOOP */
    tau_ref = /*k_des*ref[0] - */ - Kdes*(theta - ref[0]) -  Ddes*dtheta_filt - Ides*ddtheta_filt;

    float x = hw->get_theta(0);

    double inv_model_num[6] = {(307928880*Bl + 2463431040000*Jl + 538860857),  
    (541382571 - 7378496640000*Jl - 306454320*Bl)  , 
    (7366968960000*Jl - 307895280*Bl - 533817429)
     , 306487920*Bl - 2451903360000*Jl - 536339143
     , 0, 0};

    double inv_model_den[6] = {(542984400*Bl + 4343875200000*Jl) , 
    (- 532198800*Bl - 12945340800000*Jl)
      ,(12859324800000*Jl - 542950800*Bl) , 
     532232400*Bl - 4257859200000*Jl
      , 0, 0};

    //double filter_num[6] = {0,0.0001233,0.0001217,0,0,0};
    //double filter_den[6] = {1, -1.961,0.9608,0,0,0};

    //double filter_num[6] = {0.0226E-3,0.2007E-3,0.0218E-3,0,0,0};
    //double filter_den[6] = {1, -1.9605,0.9608,0,0,0};


    double filter_num[6] = {0 ,  0.310425427160331E-4  , 0.3083628091595823E-4 ,0,0,0};
    double filter_den[6] = {1.000000000000000 , -1.980136794483123  , 0.980198673306755,0,0,0};
    //double filter_den[6] = {0, 0,0,0,0,0};

    double inv_model_exit = 0;

    double filter_exit = 0;

    if(hw->get_current_time() > 0){

        inv_model_exit = 
        inv_model_num[0]*hw->get_tau_s(1) + 
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
        filter_num[0]*(hw->get_tau_m(0)) + 
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
        controller_prev1_tauM = hw->get_tau_m(0);

        controller_prev6_tauSensor = controller_prev5_tauSensor;
        controller_prev5_tauSensor = controller_prev4_tauSensor;
        controller_prev4_tauSensor = controller_prev3_tauSensor;
        controller_prev3_tauSensor = controller_prev2_tauSensor;
        controller_prev2_tauSensor = controller_prev1_tauSensor;
        controller_prev1_tauSensor = hw->get_tau_s(1);
    }



    reference = tau_ref;
    //tau = hw->get_tau_s(1);     // was 0: tauS
    //dtau = hw->get_d_tau_s(1);  // was 0: tauS

    //tau = lowPass->process(hw->get_tau_s(1), hw->get_dt());
    //dtau = lowPassD->process(hw->get_d_tau_s(1), hw->get_dt());

    tau = hw->get_tau_s(1);
    dtau = hw->get_d_tau_s(1);

    err = reference - tau;

    derr = (2.45*err - 6*prev1_err + 7.5*prev2_err - 6.66*prev3_err 
    + 3.75*prev4_err - 1.2*prev5_err + 0.16*prev6_err)/
    (hw->get_dt());


    ierr += err * hw->get_dt();
    errPast = err;

    *(hw->fric1) = inv_model_exit;
    *(hw->fric2) = filter_exit;

    float out = kp * err + kd * derr + ki * ierr;

    double dob_exit = inv_model_exit - filter_exit;

    int a = 45;

    if(dob_exit > a){
        dob_exit = a;
    }
    if(dob_exit < -a){
        dob_exit = -a;
    }

    prev6_err = prev5_err;
    prev5_err = prev4_err;
    prev4_err = prev3_err;
    prev3_err = prev2_err;
    prev2_err = prev1_err;
    prev1_err = err;


    return (out - DobGain*dob_exit);
}