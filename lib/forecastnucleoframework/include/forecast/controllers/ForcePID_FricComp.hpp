#ifndef FORCE_PID_FRICCOMP_H
#define FORCE_PID_FRICCOMP_H

#include <utility/filters/AnalogFilter.hpp>
#include "../Controller.hpp"

namespace forecast {

/**
 * @brief ForcePID control class
 **/

class ForcePID_FricComp : public Controller {
   public:

    /**
     * @brief Construct a new Force P I D object. This constructor initialize,
     * the controller.
     *
     * @param kp
     * @param ki
     * @param kd
     */
    ForcePID_FricComp (float kp = 0, float ki = 0, float kd = 0, float Kspring = 0, float m1 = 0, float m2 = 0);

    virtual float process(const IHardware* hw, std::vector<float> ref) override;

   protected:
    float kp = 0.0;
    float ki = 0.0;
    float kd = 0.0;
    float Kspring = 0.0;
    float m1 = 0.0;
    float m2 = 0.0;

    float x = 0.0f;
    float dx = 0.0f;
    float ddx = 0.0f;

    float x_e = 0.0f;
    float dx_e = 0.0f;
    float ddx_e = 0.0f;

    float tau = 0.0f;
    float dtau = 0.0f;

    float err = 0.0;
    float derr = 0.0;
    float ierr = 0.0;

    float errPast = 0.0;
    float LastForce = 0.0;

    float flag_first_cycle = 0;
    float start_pos_x_e = 0;
    float start_force_spring = 0;
    float last_fric1 = 0;
    float last_fric2 = 0;

    float out;
    float reference = 0.0;
    float last_ref = 0;

    utility::AnalogFilter* lowPass;
    utility::AnalogFilter* lowPassD;
};

inline ControllerFactory::Builder  make_Force_PID_FricComp_builder() {

    auto fn = [](std::vector<float> params) -> Controller * {
        if (params.size() < 2)
            return nullptr;
        return new ForcePID_FricComp(params[0], params[1], params[2], params[3], params[4], params[5]);
    };

    return {fn, {"KP", "KI", "KD", "Kspring", "m1", "m2"}, {"reference"}};
}

}  // namespace forecast

#endif  // FORCE_PID_H