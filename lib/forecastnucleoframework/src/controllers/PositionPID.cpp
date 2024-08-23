#include <forecast/controllers/PositionPID.hpp>

using namespace forecast;

PositionPID::PositionPID(float kp, float ki, float kd, float dob_gain)
    : kp(kp), ki(ki), kd(kd), err(0.f), derr(0.f), ierr(0.f), dob_gain(dob_gain), errPast(0.f) {
  logs.push_back(&reference);

  lowPass = utility::AnalogFilter::getDifferentiatorHz(10.0f);
  lowPassExit = utility::AnalogFilter::getLowPassFilterHz(20.0f);
}

float PositionPID::process(const IHardware *hw, std::vector<float> ref) {

  double inv_model_num[6] = { 0   ,1.592230835850342 , -1.582310443952793 ,0,0 , 0};

  double inv_model_den[6] = {1.000000000000000  ,-1.984087638487695  , 0.984127320055285, 0   ,0,0};


  double filter_num[6] = {0  , 0.310425427160331E-4  , 0.308362809159582E-4 ,0,0,0};
  double filter_den[6] = {1.000000000000000  ,-1.980136794483123 ,  0.980198673306755,0,0,0};
    //double filter_den[6] = {0, 0,0,0,0,0};

  double inv_model_exit = 0;

  double filter_exit = 0;

  float tau = hw->get_tau_s(1);
  float dtau = hw->get_d_tau_s(1);
  theta = hw->get_theta(0);
  dtheta = hw->get_d_theta(0);


  //dtheta = hw->get_d_theta(0);
  reference = ref[0];
  //float tau = tau = hw->get_tau_s(1);
  err = ref[0] - theta;
  // err = 0.5;
  //derr = (err - errPast) / hw->get_dt();

  derr = (2.45*err - 6*last_erro_1 + 7.5*last_erro_2 - 6.66*last_erro_3 + 3.75*last_erro_4 - 1.2*last_erro_5 + 0.16*last_erro_6)/(hw->get_dt());
    
  derr = lowPass->process(derr,hw->get_dt());
  
  last_erro_6 = last_erro_5;
  last_erro_5 = last_erro_4;
  last_erro_4 = last_erro_3;
  last_erro_3 = last_erro_2;
  last_erro_2 = last_erro_1;
  last_erro_1 = err;

  ierr += err * hw->get_dt();

  errPast = err;

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

  out = kp * err + ki * ierr + kd * derr;

  int a = 30;

  double dob_exit = (tau + inv_model_exit) - filter_exit;

  if(dob_exit > a){
        dob_exit = a;
  }
  if(dob_exit < -a){
        dob_exit = -a;
  }
  
  *(hw->var1) = out;
  *(hw->var2) = err;
  *(hw->var3) = theta;
  *(hw->var4) = dob_exit;

  out -= dob_gain*dob_exit;

  //*(hw->fric2) = out;

  out = lowPassExit->process(out,hw->get_dt());

  *(hw->fric2) = out;

  return out;
}