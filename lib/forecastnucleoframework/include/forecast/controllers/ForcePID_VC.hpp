#ifndef FORCE_PID_VC_H
#define FORCE_PID_VC_H

#include <utility/filters/AnalogFilter.hpp>
#include "../Controller.hpp"

namespace forecast {

/**
 * @brief ForcePID VC control class
 **/

class ForcePIDVC : public Controller {
   public:

    /**
     * @brief Construct a new Force P I D object. This constructor initialize,
     * the controller.
     *
     * @param kp
     * @param ki
     * @param kd
     * @param ps // Supply (Pump) Pressure
     * 
     */
    ForcePIDVC(float kp = 0, float ki = 0, float kd = 0, float ps = 100.0E+5);

    virtual float process(const IHardware* hw, std::vector<float> ref) override;

   protected:
    float kp = 0.0;
    float ki = 0.0;
    float kd = 0.0;

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
    float Va0 = 0.0f;
    float Vb0 = 0.0f;
    float Vpl = 0.0f;
    float In = 0.0f;
    float pn = 0.0f;
    float qn = 0.0f;
    float Fv = 0.0f;
    float Wv = 0.0f;
    float Dv = 0.0f;
    float Kv = 0.0f;
    float Kvp = 0.0f;
    float Kqua = 0.0f;
    float Kqub = 0.0f;
    float Pa0 = 0.0f;
    float Pb0 = 0.0f;
    float Ps = 0.0f;
    float Pt = 0.0f;
    float Kqu = 0.0f;
    float Kd = 0.0f;
    
    utility::AnalogFilter* lowPass;
    utility::AnalogFilter* lowPassD;
    utility::AnalogFilter* lowPassx;
    utility::AnalogFilter* lowPassDx;
};

inline ControllerFactory::Builder make_Force_PID_VC_builder() {

    auto fn = [](std::vector<float> params) -> Controller * {
        if (params.size() < 4)
            return nullptr;
        return new ForcePIDVC(params[0], params[1], params[2], params[3]);
    };

    return {fn, {"KP", "KI", "KD", "Supply Pressure [Bar]"}, {"reference"}};
}

}  // namespace forecast

#endif  // FORCE_PID_VC_H