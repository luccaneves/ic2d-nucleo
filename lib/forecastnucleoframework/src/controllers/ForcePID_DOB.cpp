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

float prev1_filter1_out = 0;
float prev2_filter1_out = 0;
float prev3_filter1_out = 0;
float prev4_filter1_out = 0;
float prev5_filter1_out = 0;


float prev1_filter2_out = 0;
float prev2_filter2_out = 0;
float prev3_filter2_out = 0;
float prev4_filter2_out = 0;
float prev5_filter2_out = 0;

float ForcePID_DOB::process(const IHardware *hw, std::vector<float> ref)
{
    //Lucca, pq vc n fez um define? Preguiça
    int freq_selector = 1; //0: 1000 , 1:2000, 2:3000

    /*if(freq_selector == 0){
        float filter_taum_coef[6] = {0,0,0,0,0,0};
        float filter_taus_coef[6] = {0,0,0,0,0,0};
        float filter_1_coef[6] = {0,0,0,0,0,0};
        float filter_2_coef[6] = {0,0,0,0,0,0};
    }
    else if(freq_selector == 1){*/
        float filter_taum_coef[6] = {0.00000982,0.00001965,0.00000982,0,0,0};
        float filter_taus_coef[6] = {0.0000289,0.00000115,-0.0000566,-0.00000115,0.00002774,0};
        float filter_1_coef[6] = {1.991,-0.0012,0,0,0,0};
        float filter_2_coef[6] = {-0.008886,1.991,-0.008807,-0.9912,0,0};
    /*}
    else{
        float filter_taum_coef[6] = {0,0,0,0,0,0};
        float filter_taus_coef[6] = {0,0,0,0,0,0};
        float filter_1_coef[6] = {0,0,0,0,0,0};
        float filter_2_coef[6] = {0,0,0,0,0,0};
    }*/

    float filter1_exit = 0;
    prev1_filter1_out = 0;
    prev2_filter1_out = 0;
    prev3_filter1_out = 0;
    prev4_filter1_out = 0;
    prev5_filter1_out = 0;


    float filter2_exit = 0;
    prev1_filter2_out = 0;
    prev2_filter2_out = 0;
    prev3_filter2_out = 0;
    prev4_filter2_out = 0;
    prev5_filter2_out = 0;

    filter1_exit = hw->get_tau_m(0)*filter_taum_coef[0] + hw->prev1_tauM*filter_taum_coef[1] + filter_taum_coef[2]*hw->prev2_tauM + prev1_filter1_out*filter_1_coef[0] + filter_1_coef[1]*prev1_filter2_out;

    prev5_filter1_out = prev4_filter1_out;
    prev4_filter1_out = prev3_filter1_out;
    prev3_filter1_out = prev2_filter1_out;
    prev2_filter1_out = prev1_filter1_out;
    prev1_filter1_out = filter1_exit;

    filter2_exit = hw->get_tau_s(1)*filter_taus_coef[0] + hw->prev1_tauSensor*filter_taus_coef[1] + hw->prev2_tauSensor*filter_taus_coef[2] + hw->prev3_tauSensor*filter_taus_coef[3] + hw->prev4_tauSensor*filter_taus_coef[4] + filter_2_coef[0]*prev1_filter2_out + filter_2_coef[1]*prev2_filter2_out + filter_2_coef[2]*prev3_filter2_out + filter_2_coef[3]*prev4_filter2_out;

    prev5_filter2_out = prev4_filter2_out;
    prev4_filter2_out = prev3_filter2_out;
    prev3_filter2_out = prev2_filter2_out;
    prev2_filter2_out = prev1_filter2_out;
    prev1_filter2_out = filter2_exit;

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

    out = ref[0] + kp * err + kd * derr + ki * ierr;



    return out + filter1_exit - filter2_exit;
}