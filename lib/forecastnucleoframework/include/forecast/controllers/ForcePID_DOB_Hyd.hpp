#ifndef FORCE_PID_DOB_HYD_H
#define FORCE_PID_DOB_HYD_H

#include <utility/filters/AnalogFilter.hpp>
#include "../Controller.hpp"

namespace forecast {

/**
 * @brief ForcePID control class
 **/

class ForcePID_DOB_HYD : public Controller {
   public:

    /**
     * @brief Construct a new Force P I D object. This constructor initialize,
     * the controller.
     *
     * @param kp
     * @param ki
     * @param kd
     *  * @param Kvc
  * @param Kpc
  * @param lambda
  * @param B_int
  * @param Ml
  * @param Bl
  * @param Kl
  * @param gain_dob
  * @param limit_dob
  * @param limit
     */
    ForcePID_DOB_HYD(float kp = 0, float ki = 0, float kd = 0, float Kvc = 0,
                   float Kpc = 0, float lambda = 0, float B_int = 0, float Ml = 0, float Bl = 0, float Kl = 0, 
                   float gain_dob = 0,float limit_dob = 0, float limit = 0);

    virtual float process(const IHardware* hw, std::vector<float> ref) override;

   protected:
    float kp = 0.0;
    float ki = 0.0;
    float kd = 0.0;
    float kpc = 0;
    float kvc = 0;
    float Lambda = 0;

    float Ml = 0;
    float Bl = 0;
    float Kl = 0;

    float tau = 0.0f;
    float dtau = 0.0f;

    float err = 0.0;
    float derr = 0.0;
    float ierr = 0.0;

    float errPast = 0.0;

    float out;
    float reference = 0.0;

    float gain_dob = 0;

    float limit_dob = 0;
    float limit = 0;

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
    double controller_prev7_tauSensor;



    double controller_prev1_tauM;
    double controller_prev2_tauM;
    double controller_prev3_tauM;
    double controller_prev4_tauM;
    double controller_prev5_tauM;
    double controller_prev6_tauM;

    float prev1_ddx = 0;
    float prev1_dx = 0;
    float prev1_dsensor = 0;

    float prev1_err = 0;
    float prev2_err = 0;
    float prev3_err = 0;
    float prev4_err = 0;
    float prev5_err = 0;
    float prev6_err = 0;

    float prev1_z = 0;
    float prev2_z = 0;
    float prev3_z = 0;
    float prev4_z = 0;
    float prev5_z = 0;
    float prev6_z = 0;

    float prev1_disturb = 0;
    float prev2_disturb = 0;
    float prev3_disturb = 0;
    float prev4_disturb = 0;
    float prev5_disturb = 0;
    float prev6_disturb = 0;

    float deriv_force = 0;

    float prev1_force = 0;
    float prev2_force = 0;
    float prev3_force = 0;
    float prev4_force = 0;
    float prev5_force = 0;
    float prev6_force = 0;

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
    float Pa = 0.0f;
    float Pb = 0.0f;
    float Kqu = 0.0f;
    float Kd = 0.0f;
    float f = 0;
    float B_int = 0;

    float prev_out = 0;

    utility::AnalogFilter* lowPass;
    utility::AnalogFilter* lowPassD;
    utility::AnalogFilter* lowPassPs;
    utility::AnalogFilter* lowPassPt;
    utility::AnalogFilter* lowPassPa;
    utility::AnalogFilter* lowPassPb;
};

inline ControllerFactory::Builder make_Force_PID_DOB_hyd_builder() {

    auto fn = [](std::vector<float> params) -> Controller * {
        if (params.size() < 13)
            return nullptr;
        return new ForcePID_DOB_HYD(params[0], params[1], params[2], params[3],
                                params[4],params[5],params[6],params[7],params[8],params[9],params[10],params[11],params[12]);
    };

    return {fn, {"KP", "KI", "KD","gain_f","gain_g","lambda","B_int","Ml","Bl","Kl","gain_dob","limit_dob","limit"}, {"reference"}};
}

}  // namespace forecast

#endif  // FORCE_PID_H