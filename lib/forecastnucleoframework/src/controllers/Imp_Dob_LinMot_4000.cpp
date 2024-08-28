#include <forecast/controllers/Imp_Dob_LinMot_4000.hpp>

using namespace forecast;

Imp_Dob_LinMot_4000::Imp_Dob_LinMot_4000(float kp, float ki, float kd,float Ides, float Ddes, float Kdes, float DobGain, float vc_gain, float jm, float bm)
    : kp(kp),
      ki(ki),
      kd(kd),
      Kdes(Kdes),
      Ddes(Ddes),
      Ides(Ides),
      VC_gain(vc_gain),
      Jm(jm),
      Bm(bm),
      DobGain(DobGain),
      errPast(0.f),
      err(0.f),
      derr(0.f),
      ierr(0.f)
{

    logs.push_back(&reference);
    lowPass = utility::AnalogFilter::getLowPassFilterHz(20.0f);
    lowPassD = utility::AnalogFilter::getLowPassFilterHz(15.0f);
    lowPassDD = utility::AnalogFilter::getLowPassFilterHz(15.0f);
    lowPassDForce = utility::AnalogFilter::getLowPassFilterHz(15.0f);
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
    tau_ref = /*k_des*ref[0] - */ - Kdes*((theta - theta_eq) - ref[0]) -  Ddes*dtheta - Ides*ddtheta;

    *(hw->fric1) = (theta - theta_eq);

    float x = hw->get_theta(0);

    float compensate_1 = ((ddtheta*Jm)) + (dtheta*Bm);

    float compensate_2 = 0;

    float compensate_3 = 0;

    *(hw->vel_comp_value) = VC_gain*( compensate_2  + compensate_1);


    //double inv_model_num[6] = {0.571428571428571,  -1.707480636386558  , 1.700840103589048 , -0.564787194477277,0 , 0};
    //double inv_model_den[6] = {1.000000000000000 , -2.973906285106519  , 2.947998206924892  ,-0.974091536281782, 0, 0};

    double inv_model_num[6] = { 0   ,1.592230835850342 , -1.582310443952793 ,0,0 , 0};

    double inv_model_den[6] = {1.000000000000000  ,-1.984087638487695  , 0.984127320055285, 0   ,0,0};


    double filter_num[6] = {0  , 0.310425427160331E-4  , 0.308362809159582E-4 ,0,0,0};
    double filter_den[6] = {1.000000000000000  ,-1.980136794483123 ,  0.980198673306755,0,0,0};
    //double filter_den[6] = {0, 0,0,0,0,0};

    double inv_model_exit = 0;

    double filter_exit = 0;

    if(hw->get_current_time() > 0){
        float value = hw->get_tau_s(1);
        value = dtheta;

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



    //reference = tau_ref;
    reference = ref[0];
    //tau = hw->get_tau_s(1);     // was 0: tauS
    //dtau = hw->get_d_tau_s(1);  // was 0: tauS

    //tau = lowPass->process(hw->get_tau_s(1), hw->get_dt());
    //dtau = lowPassD->process(hw->get_d_tau_s(1), hw->get_dt());

    tau = hw->get_tau_s(1);
    dtau = hw->get_d_tau_s(1);

    err = tau_ref - tau;

    derr = (2.45*err - 6*prev1_err + 7.5*prev2_err - 6.66*prev3_err 
    + 3.75*prev4_err - 1.2*prev5_err + 0.16*prev6_err)/
    (hw->get_dt());


    //derr = lowPassDForce->process(derr,hw->get_dt());


    ierr += err * hw->get_dt();
    errPast = err;

    
    //*(hw->fric2) = filter_exit;

    float out = kp * err + kd * derr + ki * ierr + tau_ref;

    double dob_exit = (tau + inv_model_exit) - filter_exit;

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


    float control_output_no_filter = out - DobGain*dob_exit + VC_gain*(compensate_1);

    float out_control = lowPass->process(control_output_no_filter, hw->get_dt());

    *(hw->fric2) = out_control;

    return (out_control);
}