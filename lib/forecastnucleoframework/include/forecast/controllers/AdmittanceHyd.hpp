#ifndef ADM_HYD_H
#define ADM_HYD_H

#include <utility/filters/AnalogFilter.hpp>

#include "../Controller.hpp"

namespace forecast {

/**
 * @brief Feedback Linearization Control class
 **/
class AdmittanceHyd : public Controller {
public:
  /**
   * @brief Construct a new Feedback Linearization object. This constructor
   initialize,
  * the controller.
  * @param kp
  * @param kd
  * @param ki
  * @param Kvc
  * @param Kpc
  * @param B_int
  * @param leak_fix
  * @param limit

  **/
  AdmittanceHyd(float kp = 0, float kd = 0, float ki = 0, float Kdes = 0, float Bdes = 0, float Mdes = 0);

  virtual float process(const IHardware *hw, std::vector<float> ref) override;

  float PositionController(const IHardware *hw, float ref);


protected:

  float Kdes = 0;
  float Bdes = 0;
  float Mdes = 0;

  float kp = 0.0;
  float kd = 0.0;
  float ki = 0.0f;
  float offset_x = 0;
  float once = 1;
  float once_force = 1;
  float filter_out = 0;

  float d_error = 0;
  float last_error = 0;

  float err_adm;
  float derr_adm;

  float ierr_adm;

  float theta_ref = 0;

  float gain_out = 0;

  float Kvc = 0.0f;
  float Kpc = 0.0f;
  float Ml = 0;
  float Kl = 0;

  float last_erro_1 = 0;
  float last_erro_2 = 0;
  float last_erro_3 = 0;
  float last_erro_4 = 0;
  float last_erro_5 = 0;
  float last_erro_6 = 0;

  float tau = 0.0f;
  float dtau = 0.0f;
  float x = 0.0f;
  float dx = 0.0f;
  float ddx = 0;
  float z = 0;
  float aux1 = 0;
  float aux2 = 0;

  float prev_ref_1 = 0.0;
  float prev_ref_2 = 0.0;
  float prev_ref_3 = 0.0;
  float prev_ref_4 = 0.0;
  float prev_ref_5 = 0.0;
  float prev_ref_6 = 0.0;
  float prev_ref_7 = 0.0;

  float err = 0.0;
  float derr = 0.0;
  float ierr = 0.0;

  float errPast = 0.0;

  float out;
  float reference = 0.0;

  float Be = 0.0f;
  float De = 0.0f;
  float Dh = 0.0f;
  float L_cyl = 0.0f;
  float Aa = 0.0f;
  float Ap = 0.0f;
  float Ab = 0.0f;
  float alfa = 0.0f;
  float Va = 0.0f;
  float Vb = 0.0f;
  float Vpl = 0.0f;
  float In = 0.0f;
  float pn = 0.0f;
  float qn = 0.0f;
  float Fv = 0.0f;
  float Kv = 0.0f;
  float Ps = 0.0f;
  float Pt = 0.0f;
  float Pa = 0.0f;
  float Pb = 0.0f;
  float f = 0.0f;
  float g = 0.0f;
  float v = 0.0f;
  float ixv = 0.0f;
  float B_int = 0;
  float leak_fix = 0;
  float limit = 0;
  float start_x = 0;
  float d_force = 0;

  float prev_erro_1 = 0.0;
  float prev_erro_2 = 0.0;
  float prev_erro_3 = 0.0;
  float prev_erro_4 = 0.0;
  float prev_erro_5 = 0.0;
  float prev_erro_6 = 0.0;
  float prev_erro_7 = 0.0;

  double prev1_filter_exit_dx = 0;
  double prev2_filter_exit_dx = 0;
  double prev3_filter_exit_dx = 0;
  double prev4_filter_exit_dx = 0;
  double prev5_filter_exit_dx = 0;

  double prev1_dx = 0;
  double prev2_dx = 0;
  double prev3_dx = 0;
  double prev4_dx = 0;
  double prev5_dx = 0;

  double prev1_filter_exit_dF = 0;
  double prev2_filter_exit_dF = 0;
  double prev3_filter_exit_dF = 0;
  double prev4_filter_exit_dF = 0;
  double prev5_filter_exit_dF = 0;

  double prev1_dF = 0;
  double prev2_dF = 0;
  double prev3_dF = 0;
  double prev4_dF = 0;
  double prev5_dF = 0;

  double prev1_filter_exit_current = 0;
  double prev2_filter_exit_current = 0;
  double prev3_filter_exit_current = 0;
  double prev4_filter_exit_current = 0;
  double prev5_filter_exit_current = 0;

  double prev1_current = 0;
  double prev2_current = 0;
  double prev3_current = 0;
  double prev4_current = 0;
  double prev5_current = 0;



  double prev1_disturb1 = 0;
  double prev2_disturb1 = 0;
  double prev3_disturb1 = 0;
  double prev4_disturb1 = 0;
  double prev5_disturb1 = 0;
  double prev6_disturb1 = 0;



  double prev1_ref_x_1 = 0;
  double prev1_ref_x_2 = 0;
  double prev1_ref_x_3 = 0;
  double prev1_ref_x_4 = 0;
  double prev1_ref_x_5 = 0;
  double prev1_ref_x_6 = 0;

  float gain_vc = 0;
  float vc_limit = 0;

  float lambda = 0;
  float gain_dob = 0;
  float fl = 0;
  float limit_dob = 0;
  float disturb = 0.000;
  float disturb1 = 0.000;
  float disturb2 = 0.000;
  float disturb3 = 0.000;

  float deriv_ref = 0;
  float deriv_erro = 0;

  float expected_force = 0.0;
  float last_out = 0.0;
  float dob_formulation = 0;
  float pressure_predict = 0;

  utility::AnalogFilter* lowPass;
  utility::AnalogFilter* lowPassD;
  utility::AnalogFilter* lowPassx;
  utility::AnalogFilter* lowPassDx;
  utility::AnalogFilter* lowPassPs;
  utility::AnalogFilter* lowPassPt;
  utility::AnalogFilter* lowPassPa;
  utility::AnalogFilter* lowPassPb;
  utility::AnalogFilter *admittanceTF;
  utility::AnalogFilter *lowPass_DerivRef;
  utility::AnalogFilter *lowPass_DerivErroADM;
};

inline ControllerFactory::Builder make_AdmHyd_builder() {

  auto fn = [](std::vector<float> params) -> Controller * {
    if (params.size() < 1)
      return nullptr; // not enough parameters

    return new AdmittanceHyd(params[0], params[1], params[2], params[3], params[4], params[5]);
  };

  return {
      fn,
      {"Kp", "Kd", "Ki","Kdes","Bdes","Mdes"},
      {"reference"}};
}

} // namespace forecast

#endif // IFEEDBACK_LIN_H
