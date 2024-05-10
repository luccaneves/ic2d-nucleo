#ifndef ControlForceByPress
#define ControlForceByPress

#include <utility/filters/AnalogFilter.hpp>
#include "../Controller.hpp"

namespace forecast {

/**
 * @brief ForcePID control class
 **/

class ControlForceByPress : public Controller {
   public:

    /**
     * @brief Construct a new Force P I D object. This constructor initialize,
     * the controller.
     *
     * @param kp
     * @param ki
     * @param kd
     */
    ControlForceByPress(float kp_force = 0, float ki_force = 0, float kd_force = 0,
    float kp_press = 0, float ki_press = 0, float kd_press = 0);

    virtual float process(const IHardware* hw, std::vector<float> ref) override;

   protected:
    float kp_force = 0.0;
    float ki_force = 0.0;
    float kd_force = 0.0;

    float kp_press = 0.0;
    float ki_press = 0.0;
    float kd_press= 0.0;

    float tau = 0.0f;
    float dtau = 0.0f;

    float err_tau;
    float derr_tau;
    float ierr_tau;


    float err = 0.0;
    float derr = 0.0;
    float ierr = 0.0;

    float errPast_tau = 0;
    float errPast = 0.0;

    float Pa = 0.0;
    float Pb = 0.0;
    float Pl = 0.0;

    float out_tau;
    float out;
    float reference = 0.0;

    utility::AnalogFilter* lowPass;
    utility::AnalogFilter* lowPassD;
    utility::AnalogFilter* lowPassPa;
    utility::AnalogFilter* lowPassPb;
};

inline ControllerFactory::Builder make_ControlForceByPress_builder() {

    auto fn = [](std::vector<float> params) -> Controller * {
        if (params.size() < 5)
            return nullptr;
        return new ControlForceByPress(params[0], params[1], params[2]
        , params[3], params[4], params[5]);
    };

    return {fn, {"KP_force", "KI_force", "KD_force","KP_press", "KI_press", "KD_press"}, {"reference"}};
}

}  // namespace forecast

#endif  // FORCE_PID_H