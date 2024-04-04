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
     */
    ForcePID_DOB(float kp = 0, float ki = 0, float kd = 0);

    virtual float process(const IHardware* hw, std::vector<float> ref) override;

   protected:
    float kp = 0.0;
    float ki = 0.0;
    float kd = 0.0;

    float tau = 0.0f;
    float dtau = 0.0f;

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
};

inline ControllerFactory::Builder make_Force_PID_DOB_builder() {

    auto fn = [](std::vector<float> params) -> Controller * {
        if (params.size() < 2)
            return nullptr;
        return new ForcePID_DOB(params[0], params[1], params[2]);
    };

    return {fn, {"KP", "KI", "KD"}, {"reference"}};
}

}  // namespace forecast

#endif  // FORCE_PID_H