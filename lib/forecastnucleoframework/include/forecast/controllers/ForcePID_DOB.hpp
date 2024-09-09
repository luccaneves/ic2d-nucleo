#ifndef FORCE_PID_DOB_H
#define FORCE_PID_DOB_H

#include <utility/filters/AnalogFilter.hpp>
#include "../Controller.hpp"

namespace forecast {

/**
 * @brief ForcePID control class
 **/

class ForcePID_DOB : public Controller {
   public:

    /**
     * @brief Construct a new Force P I D object. This constructor initialize,
     * the controller.
     *
     * @param kp
     * @param ki
     * @param kd
     * @param Jl
     * @param Bl
     * @param Kl
     */

    ForcePID_DOB(float kp = 0, float ki = 0, float kd = 0, float GainDOB = 0, float GainVC = 0,float Jm = 0, float Dm = 0, float Ke = 0);

    virtual float process(const IHardware* hw, std::vector<float> ref) override;

   protected:
    float kp = 0.0;
    float ki = 0.0;
    float kd = 0.0;

    float Kl;
    float GainDOB;
    float GainVC;

    float K_motor;
    float K_e = 9;
    float Resist = 3;

    float Jm = 0;
    float Dm = 0;

    float tau = 0.0f;
    float dtau = 0.0f;

    float err = 0.0;
    float derr = 0.0;
    float ierr = 0.0;

    float time_start = 0;
    float time_end = 0;
    float rise_time = 0;
    float once_start = 1;
    float once_end = 1;
    float max_tau = 0;

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

    double controller_prev1_tauSensor= 0;
    double controller_prev2_tauSensor= 0;
    double controller_prev3_tauSensor= 0;
    double controller_prev4_tauSensor= 0;
    double controller_prev5_tauSensor= 0;
    double controller_prev6_tauSensor= 0;



    double controller_prev1_tauM= 0;
    double controller_prev2_tauM= 0;
    double controller_prev3_tauM= 0;
    double controller_prev4_tauM= 0;
    double controller_prev5_tauM= 0;
    double controller_prev6_tauM= 0;

    float prev1_err = 0;
    float prev2_err = 0;
    float prev3_err = 0;
    float prev4_err = 0;
    float prev5_err = 0;
    float prev6_err = 0;

    utility::AnalogFilter* lowPass;
    utility::AnalogFilter* lowPassD;
    utility::AnalogFilter* lowPassDForce;
    utility::AnalogFilter* lowPassControl;
};

inline ControllerFactory::Builder make_Force_PID_DOB_builder() {

    auto fn = [](std::vector<float> params) -> Controller * {
        if (params.size() < 8)
            return nullptr;
        return new ForcePID_DOB(params[0], params[1], params[2], params[3], params[4],params[5],params[6],params[7]);
    };

    return {fn, {"KP", "KI", "KD", "GainDOB", "GainVC","Jm","Dm","Ke"}, {"reference"}};
}

}  // namespace forecast

#endif  // FORCE_PID_H