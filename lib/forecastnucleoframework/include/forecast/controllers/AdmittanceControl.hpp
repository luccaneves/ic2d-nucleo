#ifndef ADMITTANCE_CONTROL_H
#define ADMITTANCE_CONTROL_H

#include <utility/filters/AnalogFilter.hpp>

#include "../Controller.hpp"
#include <eigen.h>

namespace forecast {

/**
 * @brief Admittance control class
 **/
class AdmittanceControl : public Controller {
public:
  /**
   * @brief Admittance control constructor
   **/
  AdmittanceControl();
  /**
   * @brief Construct a new AdmittanceControl object. This constructor
  initialize,
   * the controller.
   * @param kp
   * @param kd
   * @param k_des
   * @param b_des
   * @param j_des

  **/
  AdmittanceControl(float kp, float kd,float ki, float k_des, float b_des, float j_des,float dob_gain);

  /**
   * @brief Admittance Control process function
   * 
   * @param hw    Hardware instance
   * @param ref   Reference value
   * @return float Output
   */
  virtual float process(const IHardware *hw, std::vector<float> ref) override;


  /**
   * @brief Admittance Control process function (Eigen3 test)
   * 
   * @param hw    Hardware instance
   * @param ref   Reference value
   * @return float Output
   */
  float process_mtx(const IHardware *hw, std::vector<float> ref);

protected:
  float kp = 0.0f;
  float kd = 0.0f;
  float ki = 0;

  float k_des = 0.0f;
  float b_des = 0.0f;
  float j_des = 0.0f;

  float reference = 0.0;

  float dob_gain = 0;

  double prev1_inv_model_exit = 0;
  double prev2_inv_model_exit = 0;
  double prev3_inv_model_exit = 0;
  double prev4_inv_model_exit = 0;
  double prev5_inv_model_exit = 0;


  double prev1_filter_exit = 0;
  double prev2_filter_exit = 0;
  double prev3_filter_exit = 0;
  double prev4_filter_exit = 0;
  double prev5_filter_exit = 0;

  double controller_prev1_tauSensor;
  double controller_prev2_tauSensor;
  double controller_prev3_tauSensor;
  double controller_prev4_tauSensor;
  double controller_prev5_tauSensor;
  double controller_prev6_tauSensor;



  double controller_prev1_tauM;
  double controller_prev2_tauM;
  double controller_prev3_tauM;
  double controller_prev4_tauM;
  double controller_prev5_tauM;
  double controller_prev6_tauM;


  utility::AnalogFilter *lowPassExit;

  float tau = 0.0f;
  float dtau = 0.0f;
  float theta = 0.0f;
  float dtheta = 0.0f;
  float ddtheta = 0.0f;
  float dtheta_filt = 0.0f;
  float ddtheta_filt = 0.0f;

  float tau_err = 0.0f;
  float theta_ref = 0.0f;

  float err = 0.0f;
  float derr = 0.0f;
  float dderr = 0.0f;

  float err_past = 0.0;

  float last_erro_1 = 0;
  float last_erro_2 = 0;
  float last_erro_3 = 0;
  float last_erro_4 = 0;
  float last_erro_5 = 0;
  float last_erro_6 = 0;

  float err_adm = 0.0;
  float derr_adm = 0.0;
  float ierr_adm = 0.0;

  float out;

  utility::AnalogFilter *lowPassDTheta;
  utility::AnalogFilter *lowPassDDTheta;
  utility::AnalogFilter *admittanceTF;
};

/**
 * @brief Builder function for the add method of App class
 * 
 * @return ControllerFactory::Builder 
 */
inline ControllerFactory::Builder make_admittance_control_builder() {

  auto fn = [](std::vector<float> params) -> Controller * {
    if (params.size() < 1)
      return nullptr; // not enough parameters

    return new AdmittanceControl(params[0], params[1], params[2], params[3],
                                 params[4], params[5], params[6]);
  };

  return {
      fn,
      {"Kp", "Kd","Ki", "K_des", "B_des", "J_des","dob_gain"},
      {"tau_err", "theta_ref", "err", "derr", "dtheta_filt", "ddtheta_filt"}};
}
} // namespace forecast

#endif // ADMITTANCE_CONTROL_H
