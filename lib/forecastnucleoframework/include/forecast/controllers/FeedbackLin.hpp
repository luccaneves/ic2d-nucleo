#ifndef FEEDBACK_LIN_H
#define FEEDBACK_LIN_H

#include <utility/filters/AnalogFilter.hpp>

#include "../Controller.hpp"

namespace forecast {

/**
 * @brief Feedback Linearization Control class
 **/
class FeedbackLin : public Controller {
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

  **/
  FeedbackLin(float kp = 0, float kd = 0, float ki = 0, float Kvc = 0,
                   float Kpc = 0);

  virtual float process(const IHardware *hw, std::vector<float> ref) override;

protected:
  float kp = 0.0;
  float kd = 0.0;
  float ki = 0.0f;

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

  utility::AnalogFilter* lowPass;
  utility::AnalogFilter* lowPassD;
  utility::AnalogFilter* lowPassx;
  utility::AnalogFilter* lowPassDx;
  utility::AnalogFilter* lowPassPa;
  utility::AnalogFilter* lowPassPb;
};

inline ControllerFactory::Builder make_feedback_lin_builder() {

  auto fn = [](std::vector<float> params) -> Controller * {
    if (params.size() < 5)
      return nullptr; // not enough parameters

    return new FeedbackLin(params[0], params[1], params[2], params[3],
                                params[4]);
  };

  return {
      fn,
      {"Kp", "Kd", "Ki", "Kvc", "Kpc"},
      {"reference"}};
}

} // namespace forecast

#endif // IFEEDBACK_LIN_H
