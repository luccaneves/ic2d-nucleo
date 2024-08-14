#ifndef Impedance_Admitance_Switch
#define Impedance_Admitance_Switch

#include <utility/filters/AnalogFilter.hpp>

#include "../Controller.hpp"

namespace forecast {

/**
 * @brief Impedance Control class
 **/
class Impedance_Admitance_Switch : public Controller {
public:
  /**
   * @brief Construct a new ImpedanceControl object. This constructor
   initialize,
  * the controller.
  * @param kp
  * @param kd
  * @param k_des
  * @param b_des
  * @param j_des

  **/

  Impedance_Admitance_Switch(float kp = 0, float ki = 0, float kd = 0, float Ides = 0,
                   float Ddes = 0, float Kdes = 0, float DobGain = 0, float VC_gain = 0, float Jm = 0, float Bm = 0, float Kp_pos = 0, float Kd_pos = 0, float Ki_pos = 0, float switch_method = 0);

  virtual float process(const IHardware *hw, std::vector<float> ref) override;
   protected:
    float kp = 0.0;
    float ki = 0.0;
    float kd = 0.0;

    float Ides = 0.0;
    float Ddes = 0.0;
    float Kdes = 0.0;

    float Jl = 0.0;
    float Bl = 0.0;
    float Kl = 0.0;

    float flag_impedance_admitance = 0;
    float DobGain = 0;

    float VC_gain = 0;

    float Jm = 0;
    float Bm = 0;

    float tau = 0.0f;
    float dtau = 0.0f;
    float theta = 0.0f;
    float dtheta = 0.0f;
    float ddtheta = 0.0f;
    float dtheta_filt = 0.0f;
    float ddtheta_filt = 0.0f;
    bool once = true;
    float tau_ref = 0.0f;
    float theta_eq = 0.0f;

    float err = 0.0;
    float derr = 0.0;
    float ierr = 0.0;

    float errPast = 0.0;

    float out;
    float reference = 0.0;

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

    float Kp_pos = 0;
    float Kd_pos = 0;
    float Ki_pos = 0;
    float switch_method = 0;



    double controller_prev1_tauM;
    double controller_prev2_tauM;
    double controller_prev3_tauM;
    double controller_prev4_tauM;
    double controller_prev5_tauM;
    double controller_prev6_tauM;

    float prev1_err = 0;
    float prev2_err = 0;
    float prev3_err = 0;
    float prev4_err = 0;
    float prev5_err = 0;
    float prev6_err = 0;

    

    utility::AnalogFilter* lowPass;
    utility::AnalogFilter* lowPassD;
    utility::AnalogFilter* lowPassDD;
    utility::AnalogFilter* lowPassDForce;
    utility::AnalogFilter* lowPass_PosDeriv;
    utility::AnalogFilter* lowPassDTheta;
    utility::AnalogFilter* lowPassDDTheta;
    utility::AnalogFilter* admittanceTF;
};

inline ControllerFactory::Builder make_impedance_admitance_control_builder() {

    auto fn = [](std::vector<float> params) -> Controller * {
        if (params.size() < 1)
            return nullptr;
        return new Impedance_Admitance_Switch(params[0], params[1], params[2]
        , params[3], params[4], params[5],params[6],params[7],params[8],params[9],
        params[10],params[11],params[12],params[13]);
    };

    return {fn, {"KP", "KI", "KD","Ides","Bdes","Kdes","DOB_GAIN","VC_gain","jm","bm","Kp_pos","Kd_pos","Ki_pos","Switch_method"}, {"reference"}};
}

} // namespace forecast

#endif // IMPEDANCE_CONTROL_H
