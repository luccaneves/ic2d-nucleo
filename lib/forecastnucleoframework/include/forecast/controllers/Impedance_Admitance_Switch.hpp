#ifndef Impedance_Admitance_Switch_h
#define Impedance_Admitance_Switch_h

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

  Impedance_Admitance_Switch(float kp = 0, float ki = 0, float kd = 0, 
  float Ides = 0, float Ddes = 0, float Kdes = 0, 
  float DobGain = 0, 
  float Kp_pos = 0, float Kd_pos = 0, float Ki_pos = 0, 
  float switch_method = 0,
  float duty_delta = 0, float n_percent = 0,
  float alpha_max = 0,
  float etta_switch2 = 0, float freq_cutoff_switch2 = 0, float switch2_neg_gamma = 0, float switch2_threshold_force = 0, float switch2_delta = 0, float switch2_p = 0);

   virtual float process(const IHardware *hw, std::vector<float> ref) override;

   float Impedance_Controller(const IHardware *hw, float ref);

   float Admitance_Controller(const IHardware *hw, float ref);

   float ForceController(const IHardware *hw, float ref);
   
   float PositionController(const IHardware *hw, float ref);

   protected:
    float Kdes = 0;
    float Bdes = 0;
    float Mdes = 0;

    float kp = 0.0;
    float kd = 0.0;
    float ki = 0.0f;
    float K_env = 0;

    float Kp_pos = 0;
    float Kd_pos = 0;
    float Ki_pos = 0;

    float offset_x = 0;
    float once = 1;
    float once_force = 0;
    float filter_out = 0;

    float err_adm;
    float derr_adm;

    float ierr_adm;

    float theta_ref = 0;
    float ref_adm = 0;

    float gain_out = 0;

    float Kvc = 0.0f;
    float Kpc = 0.0f;
    float Ml = 0;
    float Kl = 0;

    float last_erro_1 = 0;
    float last_erro_2 = 0;
    float last_erro_3 = 0;
    float last_erro_4 = 0;
    float last_erro_5 = 0;
    float last_erro_6 = 0;

    float tau = 0.0f;
    float dtau = 0.0f;
    float x = 0.0f;
    float dx = 0.0f;
    float ddx = 0;
    float z = 0;
    float aux1 = 0;
    float aux2 = 0;

    float deriv_ref = 0;

    float last_erro_imp_adm = 0;

    float erro_imp_adm = 0;
    float deriv_erro_imp_adm = 0;
    float int_erro_imp_adm = 0;

    float prev_ref_1 = 0.0;
    float prev_ref_2 = 0.0;
    float prev_ref_3 = 0.0;
    float prev_ref_4 = 0.0;
    float prev_ref_5 = 0.0;
    float prev_ref_6 = 0.0;
    float prev_ref_7 = 0.0;

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
    float B_int = 0;
    float leak_fix = 0;
    float limit = 0;
    float start_x = 0;
    float d_force = 0;

    float prev_erro_1 = 0.0;
    float prev_erro_2 = 0.0;
    float prev_erro_3 = 0.0;
    float prev_erro_4 = 0.0;
    float prev_erro_5 = 0.0;
    float prev_erro_6 = 0.0;
    float prev_erro_7 = 0.0;

    double prev1_filter_exit_dx = 0;
    double prev2_filter_exit_dx = 0;
    double prev3_filter_exit_dx = 0;
    double prev4_filter_exit_dx = 0;
    double prev5_filter_exit_dx = 0;

    double prev1_dx = 0;
    double prev2_dx = 0;
    double prev3_dx = 0;
    double prev4_dx = 0;
    double prev5_dx = 0;

    double prev1_filter_exit_dF = 0;
    double prev2_filter_exit_dF = 0;
    double prev3_filter_exit_dF = 0;
    double prev4_filter_exit_dF = 0;
    double prev5_filter_exit_dF = 0;

    double prev1_dF = 0;
    double prev2_dF = 0;
    double prev3_dF = 0;
    double prev4_dF = 0;
    double prev5_dF = 0;

    double prev1_filter_exit_current = 0;
    double prev2_filter_exit_current = 0;
    double prev3_filter_exit_current = 0;
    double prev4_filter_exit_current = 0;
    double prev5_filter_exit_current = 0;

    double prev1_current = 0;
    double prev2_current = 0;
    double prev3_current = 0;
    double prev4_current = 0;
    double prev5_current = 0;



    double prev1_disturb1 = 0;
    double prev2_disturb1 = 0;
    double prev3_disturb1 = 0;
    double prev4_disturb1 = 0;
    double prev5_disturb1 = 0;
    double prev6_disturb1 = 0;



    double prev1_ref_x_1 = 0;
    double prev1_ref_x_2 = 0;
    double prev1_ref_x_3 = 0;
    double prev1_ref_x_4 = 0;
    double prev1_ref_x_5 = 0;
    double prev1_ref_x_6 = 0;

    float gain_vc = 0;
    float vc_limit = 0;

    float lambda = 0;
    float gain_dob = 0;
    float fl = 0;
    float limit_dob = 0;
    float disturb = 0.000;
    float disturb1 = 0.000;
    float disturb2 = 0.000;
    float disturb3 = 0.000;

    float expected_force = 0.0;
    float last_out = 0.0;
    float dob_formulation = 0;
    float pressure_predict = 0;

    utility::AnalogFilter* lowPass;
    utility::AnalogFilter* lowPassD;
    utility::AnalogFilter* lowPassx;
    utility::AnalogFilter* lowPassDx;
    utility::AnalogFilter* lowPassPs;
    utility::AnalogFilter* lowPassPt;
    utility::AnalogFilter* lowPassPa;
    utility::AnalogFilter* lowPassPb;
    utility::AnalogFilter *admittanceTF;

    utility::AnalogFilter *lowPass_DerivRef;
    utility::AnalogFilter *lowPass_DerivRefForce;
    utility::AnalogFilter* lowPass_DerivErroImp;
    utility::AnalogFilter* lowPass_FilterOut;


    float switch_method = 0;
    float time_start_cycle = 0;


    //Variáveis primeiro método
    float k_integr = 0;
    float duty_delta = 0;
    float n_percent = 0;
    float tempo_start = 0;

    //Variáveis segundo método
    float alpha_max = 0;

    //Variáveis terceiro método
    utility::AnalogFilter* Switch2_LowPass;
    utility::AnalogFilter* Switch2_HighPass;   
    float alpha_switch2 = 0;
    float etta_switch2 = 0;
    float freq_cutoff_switch2 = 0;
    float switch2_neg_gamma = 0;
    float switch2_threshold_force = 0;
    float switch2_Q = 0;
    float switch2_gamma = 0;
    float switch2_delta = 0;
    float switch2_p = 0;

    float flag_filter_out = 0;
    
    //Variáveis quarto método 
    float switch4_new_theta_ref = 0;








    float theta_eq = 0.0f;
    float flag_impedance_admitance = 0;
};

inline ControllerFactory::Builder make_impedance_admitance_control_builder() {

    auto fn = [](std::vector<float> params) -> Controller * {
        if (params.size() < 1)
            return nullptr;
        return new Impedance_Admitance_Switch(params[0], params[1], params[2]
        , params[3], params[4], params[5],params[6],params[7],params[8],params[9],
        params[10],params[11],params[12],params[13]
        ,params[14],params[15]
        ,params[16]
        ,params[17],params[18],params[19]
        );
    };

    return {fn, {"KP_imp", "KI_imp", "KD_imp"
    ,"Ides","Bdes","Kdes",
    "DOB_GAIN",
    "Kp_adm","Kd_adm","Ki_adm",
    "Switch_method", 
    "switch0_duty_delta","switch0_n_duty",
    "switch1_alpha_max",
    "switch2_cutoff_freq","switch2_threshold_force","switch2_etta","switch2_neg_gamma","switch2_delta","switch2_p"
    }, {"reference"}};
}

} // namespace forecast

#endif // IMPEDANCE_CONTROL_H
