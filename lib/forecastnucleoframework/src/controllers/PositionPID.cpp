#include <forecast/controllers/PositionPID.hpp>

using namespace forecast;

PositionPID::PositionPID(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd), err(0.f), derr(0.f), ierr(0.f), errPast(0.f) {
  logs.push_back(&reference);

  lowPass = utility::AnalogFilter::getDifferentiatorHz(10.0f);
}

float PositionPID::process(const IHardware *hw, std::vector<float> ref) {
  theta = hw->get_theta(1);
  //dtheta = hw->get_d_theta(0);
  reference = ref[0];

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

  out = kp * err + ki * ierr + kd * derr;

  //*(hw->fric2) = out;

  return out;
}