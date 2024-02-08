#ifndef FORCE_PID_VC_LINMOT_H
#define FORCE_PID_VC_LINMOT_H

#include <utility/filters/AnalogFilter.hpp>
#include "../Controller.hpp"

namespace forecast {

/**
 * @brief ForcePID control class
 **/

class ForcePID_VC_LinMot : public Controller {
   public:

    /**
     * @brief Construct a new Force P I D object. This constructor initialize,
     * the controller.
     *
     * @param kp
     * @param ki
     * @param kd
     * @param Jl
     * @param Dl
     * @param Kl
     */

    ForcePID_VC_LinMot(float kp = 0, float ki = 0, float kd = 0, float Jl = 0, float Dl = 0, float KL = 0);

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

    float K_motor = 17;
    float K_e = 9;
    float Resist = 3;
    float Jm = 0;
    float Dm = 0;
    float Jl = 0;
    float Dl = 0;
    float Kl = 0;
    float N = 0;

    float x = 0.0f;
    float dx = 0.0f;
    float ddx = 0.0f;

    float K = 0.5;

    utility::AnalogFilter* lowPassx;
    utility::AnalogFilter* lowPassDx;
    utility::AnalogFilter* lowPassDDx;

    float out;
    float reference = 0.0;

    utility::AnalogFilter* lowPass;
    utility::AnalogFilter* lowPassD;
};

inline ControllerFactory::Builder make_ForcePID_VC_LinMot_builder() {

    auto fn = [](std::vector<float> params) -> Controller * {
        if (params.size() < 5)
            return nullptr;
        return new ForcePID_VC_LinMot(params[0], params[1], params[2], params[3], params[4], params[5]);
    };

    return {fn, {"KP", "KI", "KD","Jl","Dl","Kl"}, {"reference"}};
}

}  // namespace forecast

#endif  // FORCE_PID_H