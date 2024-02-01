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

float prev1_inv_model_exit = 0;
float prev2_inv_model_exit = 0;
float prev3_inv_model_exit = 0;
float prev4_inv_model_exit = 0;
float prev5_inv_model_exit = 0;


float prev1_filter_exit = 0;
float prev2_filter_exit = 0;
float prev3_filter_exit = 0;
float prev4_filter_exit = 0;
float prev5_filter_exit = 0;

float ForcePID_DOB::process(const IHardware *hw, std::vector<float> ref)
{
    //Lucca, pq vc n fez um define? Preguiça
    int freq_selector = 1; //0: 1000 , 1:2000, 2:3000
    float x = hw->get_theta(0);


    float inv_model_num[6] = {0.5714, -2.28, 3.4313, -2.27, 0.5665, 0};
    float inv_model_den[6] = {1, -3.956, 5.869, -3.87, 0.9568, 0};

    float filter_num[6] = {0.0001233,0.0001217,0,0,0,0};
    float filter_den[6] = {1, -1.961,0.9608,0,0,0};

    float inv_model_exit = 0;

    float filter_exit = 0;

    float inv_model_exit = 
    inv_model_num[0]*hw->get_tau_s(1) + 
    inv_model_num[1]*hw->prev1_tauSensor  + 
    inv_model_num[2]*hw->prev2_tauSensor +
    inv_model_num[3]*hw->prev3_tauSensor +
    inv_model_num[4]*hw->prev4_tauSensor +
    inv_model_num[5]*hw->prev5_tauSensor -
    inv_model_den[1]*prev1_inv_model_exit - 
    inv_model_den[2]*prev2_inv_model_exit -
    inv_model_den[3]*prev3_inv_model_exit -
    inv_model_den[4]*prev4_inv_model_exit -
    inv_model_den[5]*prev5_inv_model_exit;

    inv_model_exit = inv_model_exit/inv_model_den[0];

    float filter_exit = 
    filter_num[0]*hw->get_tau_m(0) + 
    filter_num[1]*(hw->prev1_tauM) + 
    filter_num[2]*(hw->prev2_tauM) + 
    filter_num[3]*(hw->prev3_tauM) + 
    filter_num[4]*(hw->prev4_tauM) + 
    filter_num[5]*(hw->prev5_tauM) - 
    filter_den[1]*prev1_filter_exit - 
    filter_den[2]*prev2_filter_exit -
    filter_den[3]*prev3_filter_exit -
    filter_den[4]*prev4_filter_exit -
    filter_den[5]*prev5_filter_exit;

    filter_exit = filter_exit/ filter_den[0];



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

    *(hw->fric1) = dob_exit;

    out = ref[0] + kp * err + kd * derr + ki * ierr;



    return out + inv_model_exit - filter_exit;
}