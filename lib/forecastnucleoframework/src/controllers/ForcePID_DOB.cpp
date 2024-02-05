#include <forecast/controllers/ForcePID_DOB.hpp>

using namespace forecast;

ForcePID_DOB::ForcePID_DOB(float kp, float ki, float kd)
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
}



float ForcePID_DOB::process(const IHardware *hw, std::vector<float> ref)
{
    float x = hw->get_theta(0);


    double inv_model_num[6] = {0.571428571428570, -2.280162106468708, 3.412502702471617, -2.270230644430718, 0.566461538135732, 0};
    double inv_model_den[6] = {1, -3.956136905405657, 5.869079151390250, -3.869736663285190, 0.956794478437089, 0};

    //double filter_num[6] = {0,0.0001233,0.0001217,0,0,0};
    //double filter_den[6] = {1, -1.961,0.9608,0,0,0};

    //double filter_num[6] = {0.0226E-3,0.2007E-3,0.0218E-3,0,0,0};
    //double filter_den[6] = {1, -1.9605,0.9608,0,0,0};


    double filter_num[6] = {0,0.12233473E-3,0.1217135952E-3,0,0,0};
    double filter_den[6] = {1, -1.9605444,0.96078944,0,0,0};
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

        inv_model_exit = inv_model_exit;

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

        filter_exit = filter_exit;



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



    reference = ref[0];
    //tau = hw->get_tau_s(1);     // was 0: tauS
    //dtau = hw->get_d_tau_s(1);  // was 0: tauS

    //tau = lowPass->process(hw->get_tau_s(1), hw->get_dt());
    //dtau = lowPassD->process(hw->get_d_tau_s(1), hw->get_dt());

    tau = hw->get_tau_s(1);
    dtau = hw->get_d_tau_s(1);

    err = ref[0] - tau;
    derr = (err - errPast) / hw->get_dt();
    ierr += err * hw->get_dt();
    errPast = err;

    *(hw->fric1) = inv_model_exit;
    *(hw->fric2) = filter_exit;

    out = ref[0] + kp * err + kd * derr + ki * ierr;

    double dob_exit = inv_model_exit - filter_exit;

    int a = 80;

    if(dob_exit > a){
        dob_exit = a;
    }
    if(dob_exit < -a){
        dob_exit = -a;
    }


    return (out - dob_exit);
}