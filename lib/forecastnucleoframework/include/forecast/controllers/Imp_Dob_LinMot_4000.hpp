#ifndef Imp_Dob_LinMot_4000_h
#define Imp_Dob_LinMot_4000_h

#include <utility/filters/AnalogFilter.hpp>
#include "../Controller.hpp"

namespace forecast {

/**
 * @brief ForcePID control class
 **/

class Imp_Dob_LinMot_4000 : public Controller {
   public:

    /**
     * @brief Construct a new Force P I D object. This constructor initialize,
     * the controller.
     *
     * @param kp
     * @param ki
     * @param kd
     *  * @param Ides
  * @param Ddes
  * @param Kdes
  * @param DobGain
     */
    Imp_Dob_LinMot_4000(float kp = 0, float ki = 0, float kd = 0, float Ides = 0,
                   float Ddes = 0, float Kdes = 0, float DobGain = 0, float Jl = 0, float Bl = 0, float Kl = 0);

    virtual float process(const IHardware* hw, std::vector<float> ref) override;

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

    float DobGain = 0;


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
    utility::AnalogFilter* lowPassPs;
    utility::AnalogFilter* lowPassPt;
    utility::AnalogFilter* lowPassPa;
    utility::AnalogFilter* lowPassPb;
};

inline ControllerFactory::Builder make_Imp_Dob_LinMot_4000_builder() {

    auto fn = [](std::vector<float> params) -> Controller * {
        if (params.size() < 10)
            return nullptr;
        return new Imp_Dob_LinMot_4000(params[0], params[1], params[2]
        , params[3], params[4], params[5],params[6],params[7],params[8],params[9]);
    };

    return {fn, {"KP", "KI", "KD","Ides","Bdes","Kdes","DOB_GAIN","Jl","Bl","Kl"}, {"reference"}};
}

}  // namespace forecast

#endif  // FORCE_PID_H