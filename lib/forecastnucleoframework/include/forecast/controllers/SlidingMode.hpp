#ifndef SLIDING
#define SLIDING

#include <utility/filters/AnalogFilter.hpp>

#include "../Controller.hpp"

namespace forecast {

/**
 * @brief Feedback Linearization Control class
 **/
class SlidingMode : public Controller {
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
  SlidingMode(float max_f = 0, float min_f = 0, float max_g = 0, float min_g = 0, float etta = 0, float psi = 0, 
  float limit = 0, float gain_out = 0, float gain_dob = 0, float limit_dob = 0, float lambda = 0);

  virtual float process(const IHardware *hw, std::vector<float> ref) override;

protected:
  float kp = 0.0;
  float kd = 0.0;
  float ki = 0.0f;
  float offset_x = 0;
  float once = 1;
  float once_force = 1;

  float Kvc = 0.0f;
  float Kpc = 0.0f;

  float tau = 0.0f;
  float dtau = 0.0f;
  float x = 0.0f;
  float dx = 0.0f;

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

  float max_f = 0;
  float min_f = 0;
  float max_g = 0;
  float min_g = 0;
  float etta = 0;
  float psi = 0;
  float gain_out = 0;
  float beta = 0;
  float gain_g_med = 0;
  float gain_f_med = 0;
  float deriv_force_desejada = 0;

  float prev_erro_1 = 0.0;
  float prev_erro_2 = 0.0;
  float prev_erro_3 = 0.0;
  float prev_erro_4 = 0.0;
  float prev_erro_5 = 0.0;
  float prev_erro_6 = 0.0;
  float prev_erro_7 = 0.0;

  float prev_ref_1 = 0.0;
  float prev_ref_2 = 0.0;
  float prev_ref_3 = 0.0;
  float prev_ref_4 = 0.0;
  float prev_ref_5 = 0.0;
  float prev_ref_6 = 0.0;
  float prev_ref_7 = 0.0;

  float deriv_force_desejada = 0;

  float gain_vc = 0;
  float vc_limit = 0;

  float lambda = 0;
  float gain_dob = 0;
  float fl = 0;
  float limit_dob = 0;
  float disturb = 0;

  float expected_force = 0.0;
  float last_out = 0.0;

  utility::AnalogFilter* lowPass;
  utility::AnalogFilter* lowPassD;
  utility::AnalogFilter* lowPassx;
  utility::AnalogFilter* lowPassDx;
  utility::AnalogFilter* lowPassPs;
  utility::AnalogFilter* lowPassPt;
  utility::AnalogFilter* lowPassPa;
  utility::AnalogFilter* lowPassPb;
};

inline ControllerFactory::Builder make_SlidingMode_builder() {

  auto fn = [](std::vector<float> params) -> Controller * {
    if (params.size() < 11)
      return nullptr; // not enough parameters

    return new SlidingMode(params[0], params[1], params[2], params[3],
                                params[4],params[5],params[6],params[7],params[8],params[9],params[10]);
  };

  return {
      fn,
      {"max_f", "min_f", "max_g", "min_g", "eta","psi", "limit","gain_dob","limit_dob","lambda"},
      {"reference"}};
}

} // namespace forecast

#endif // IFEEDBACK_LIN_H
