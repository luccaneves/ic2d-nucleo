#include <forecast/controllers/Impedance_Admitance_Switch.hpp>

using namespace forecast;

Impedance_Admitance_Switch::Impedance_Admitance_Switch(float kp, float ki, float kd,float Ides, float Ddes, float Kdes, 
float DobGain, float vc_gain, float jm, float bm, float Kp_pos, float Kd_pos, float Ki_pos, float switch_method,
float duty_delta,float n_percent, 
float alpha_max,
float etta_switch2, float freq_cutoff_switch2, float switch2_neg_gamma, float switch2_threshold_force, float switch2_delta, float switch2_p
)
    : kp(kp),
      ki(ki),
      kd(kd),
      Kdes(Kdes),
      Ddes(Ddes),
      Ides(Ides),
      VC_gain(vc_gain),
      Jm(jm),
      Bm(bm),
      DobGain(DobGain),
      errPast(0.f),
      err(0.f),
      derr(0.f),
      ierr(0.f),
      Kp_pos(Kp_pos),
      Ki_pos(Ki_pos),
      Kd_pos(Kd_pos),
      switch_method(switch_method),
      duty_delta(duty_delta),
      n_percent(n_percent),
      alpha_max(alpha_max),
      etta_switch2(etta_switch2),
      freq_cutoff_switch2(freq_cutoff_switch2),
      switch2_neg_gamma(switch2_neg_gamma),
      switch2_threshold_force(switch2_threshold_force),
      switch2_delta(switch2_delta),
      switch2_p(switch2_p)

{
    logs.push_back(&reference);
    lowPass_PosDeriv = utility::AnalogFilter::getDifferentiatorHz(10.0f);
    lowPass = utility::AnalogFilter::getLowPassFilterHz(20.0f);
    lowPassD = utility::AnalogFilter::getLowPassFilterHz(15.0f);
    lowPassDD = utility::AnalogFilter::getLowPassFilterHz(15.0f);
    lowPassDForce = utility::AnalogFilter::getLowPassFilterHz(15.0f);
    lowPassDTheta = utility::AnalogFilter::getDifferentiatorHz(100.0f);
    lowPassDDTheta = utility::AnalogFilter::getDifferentiatorHz(10.0f);
    Switch2_LowPass = utility::AnalogFilter::getLowPassFilterHz(freq_cutoff_switch2);

    if(Ides > 0){
        double a_ADM[3] = {Ides,Ddes,Kdes};
        double b_ADM[3] = {0.0,0.0,1.0};   
        admittanceTF = new utility::AnalogFilter(2, a_ADM, b_ADM);
    }

    else {
        double a_ADM[2] = {Ddes,Kdes};
        double b_ADM[2] = {0.0,1.0};   
        admittanceTF = new utility::AnalogFilter(1, a_ADM, b_ADM); 
    }
}

float Impedance_Admitance_Switch::Impedance_Controller(const IHardware *hw, float ref){
     /* POSITION LOOP */
    tau_ref = /*k_des*ref[0] - */ - Kdes*((theta - theta_eq) - ref) -  Ddes*dtheta - Ides*ddtheta;

    *(hw->fric1) = (theta - theta_eq);

    float x = hw->get_theta(0);

    float compensate_1 = ((ddtheta*Jm)) + (dtheta*Bm);

    float compensate_2 = 0;

    float compensate_3 = 0;

    *(hw->vel_comp_value) = VC_gain*( compensate_2  + compensate_1);


    //double inv_model_num[6] = {0.571428571428571,  -1.707480636386558  , 1.700840103589048 , -0.564787194477277,0 , 0};
    //double inv_model_den[6] = {1.000000000000000 , -2.973906285106519  , 2.947998206924892  ,-0.974091536281782, 0, 0};

    double inv_model_num[6] = { 0   ,1.592230835850342 , -1.582310443952793 ,0,0 , 0};

    double inv_model_den[6] = {1.000000000000000  ,-1.984087638487695  , 0.984127320055285, 0   ,0,0};


    double filter_num[6] = {0  , 0.310425427160331E-4  , 0.308362809159582E-4 ,0,0,0};
    double filter_den[6] = {1.000000000000000  ,-1.980136794483123 ,  0.980198673306755,0,0,0};
    //double filter_den[6] = {0, 0,0,0,0,0};

    double inv_model_exit = 0;

    double filter_exit = 0;

    if(hw->get_current_time() > 0){
        float value = hw->get_tau_s(1);
        value = dtheta;

        inv_model_exit = 
        inv_model_num[0]*value + 
        inv_model_num[1]*controller_prev1_tauSensor + 
        inv_model_num[2]*controller_prev2_tauSensor +
        inv_model_num[3]*controller_prev3_tauSensor +
        inv_model_num[4]*controller_prev4_tauSensor +
        inv_model_num[5]*controller_prev5_tauSensor 
        -
        inv_model_den[1]*prev1_inv_model_exit - 
        inv_model_den[2]*prev2_inv_model_exit -
        inv_model_den[3]*prev3_inv_model_exit -
        inv_model_den[4]*prev4_inv_model_exit -
        inv_model_den[5]*prev5_inv_model_exit;

        inv_model_exit = inv_model_exit/inv_model_den[0];

        filter_exit = 
        filter_num[0]*(*(hw->fric2)) + 
        filter_num[1]*controller_prev1_tauM + 
        filter_num[2]*controller_prev2_tauM + 
        filter_num[3]*controller_prev3_tauM + 
        filter_num[4]*controller_prev4_tauM  
        -
        filter_den[1]*prev1_filter_exit - 
        filter_den[2]*prev2_filter_exit -
        filter_den[3]*prev3_filter_exit -
        filter_den[4]*prev4_filter_exit -
        filter_den[5]*prev5_filter_exit;

        filter_exit = filter_exit/filter_den[0];



        prev5_filter_exit = prev4_filter_exit;
        prev4_filter_exit = prev3_filter_exit;
        prev3_filter_exit = prev2_filter_exit;
        prev2_filter_exit = prev1_filter_exit;
        prev1_filter_exit = filter_exit;

        prev5_inv_model_exit = prev4_inv_model_exit;
        prev4_inv_model_exit = prev3_inv_model_exit;
        prev3_inv_model_exit = prev2_inv_model_exit;
        prev2_inv_model_exit = prev1_inv_model_exit;
        prev1_inv_model_exit = inv_model_exit;

        controller_prev6_tauM = controller_prev5_tauM;
        controller_prev5_tauM = controller_prev4_tauM;
        controller_prev4_tauM = controller_prev3_tauM;
        controller_prev3_tauM = controller_prev2_tauM;
        controller_prev2_tauM = controller_prev1_tauM;
        controller_prev1_tauM = *(hw->fric2);

        controller_prev6_tauSensor = controller_prev5_tauSensor;
        controller_prev5_tauSensor = controller_prev4_tauSensor;
        controller_prev4_tauSensor = controller_prev3_tauSensor;
        controller_prev3_tauSensor = controller_prev2_tauSensor;
        controller_prev2_tauSensor = controller_prev1_tauSensor;
        controller_prev1_tauSensor = value;
    }



    reference = tau_ref;
    //tau = hw->get_tau_s(1);     // was 0: tauS
    //dtau = hw->get_d_tau_s(1);  // was 0: tauS

    //tau = lowPass->process(hw->get_tau_s(1), hw->get_dt());
    //dtau = lowPassD->process(hw->get_d_tau_s(1), hw->get_dt());

    tau = hw->get_tau_s(1);
    dtau = hw->get_d_tau_s(1);

    err = reference - tau;

    derr = (2.45*err - 6*prev1_err + 7.5*prev2_err - 6.66*prev3_err 
    + 3.75*prev4_err - 1.2*prev5_err + 0.16*prev6_err)/
    (hw->get_dt());


    //derr = lowPassDForce->process(derr,hw->get_dt());


    ierr += err * hw->get_dt();
    errPast = err;

    
    //*(hw->fric2) = filter_exit;

    float out = kp * err + kd * derr + ki * ierr + reference;

    double dob_exit = (tau + inv_model_exit) - filter_exit;

    int a = 45;

    if(dob_exit > a){
        dob_exit = a;
    }
    if(dob_exit < -a){
        dob_exit = -a;
    }


    prev5_err = prev4_err;
    prev4_err = prev3_err;
    prev3_err = prev2_err;
    prev2_err = prev1_err;
    prev1_err = err;


    float control_output_no_filter = out - DobGain*dob_exit + VC_gain*(compensate_1);

    return control_output_no_filter;
}

float Impedance_Admitance_Switch::Admitance_Controller(const IHardware *hw, float ref){
    dtheta_filt = lowPassDTheta->process(theta, hw->get_dt());
    ddtheta_filt = lowPassDDTheta->process(dtheta, hw->get_dt());

    /* FORCE LOOP */
    tau_err = (tau);
    //Lucca: n tenho certeza disso...
    
    theta_ref = admittanceTF->process(tau_err,hw->get_dt());

    /* POSITION LOOP */
    err_adm = ref - theta_ref;
    derr_adm = (2.45*err - 6*last_erro_1 + 7.5*last_erro_2 - 6.66*last_erro_3 + 3.75*last_erro_4 - 1.2*last_erro_5 + 0.16*last_erro_6)/(hw->get_dt());

    derr_adm = lowPass->process(derr_adm,hw->get_dt());

    ierr_adm += err_adm*hw->get_dt();
    
    last_erro_6 = last_erro_5;
    last_erro_5 = last_erro_4;
    last_erro_4 = last_erro_3;
    last_erro_3 = last_erro_2;
    last_erro_2 = last_erro_1;
    last_erro_1 = err_adm;
    
    float out = Kp_pos*err_adm + Kd_pos*derr_adm + Ki_pos*ierr_adm;

    return out;
}

float Impedance_Admitance_Switch::process(const IHardware *hw, std::vector<float> ref)
{
    tau = hw->get_tau_s(1);
    dtau = hw->get_d_tau_s(1);
    theta = hw->get_theta(0);
    dtheta = hw->get_d_theta(0);
    ddtheta = hw->get_dd_theta(0);
    dtheta_filt = lowPassD->process(dtheta, hw->get_dt());
    ddtheta_filt = lowPassDD->process(ddtheta, hw->get_dt());


    /* Get the equilibrium state */
    if (once)
    {
        tempo_start =  hw->get_t();
        tau_eq = tau;
        theta_eq = theta;
        once = false;
        errPast = 0;
    }
    float time_start_cycle = 0;
    //Colocar código para definir se é impedancia ou admitancia

    if(switch_method == 0 || switch_method == 1){
        if(switch_method == 0){
            float time = hw->get_t(); //Tempo em s

            if(((time_start_cycle) <= time) 
            && 
            (time < (time_start_cycle + (1 - n_percent)*duty_delta)))
            
            {
                flag_impedance_admitance = 1;
            }

            else
            {
                flag_impedance_admitance = 0;
            }

            if(time > (time_start_cycle + duty_delta)){
                time_start_cycle = time;
                //k_integr +=1;
            }
        }

        else if(switch_method == 1){
            if(abs(ref[0] - theta) > alpha_max){
                flag_impedance_admitance = 0;
            }
            else{
                flag_impedance_admitance = 1;
            }
        }

        if(flag_impedance_admitance == 1){
            out = Impedance_Controller(hw,ref[0]);
        }
        else{

            out = Admitance_Controller(hw,ref[0]);
        }

        out = lowPass->process(out,hw->get_dt());

    }



    else{
        if(switch_method == 2){
            float tau_low = Switch2_LowPass->process(tau,hw->get_dt());
            float tau_high = Switch2_HighPass->process(tau,hw->get_dt());

            if(tau_low < switch2_threshold_force && tau_high < switch2_threshold_force){
                switch2_Q = 0;
            }
            else{
                switch2_Q = 1;
            }

            if(switch2_Q == 0){
                switch2_gamma = switch2_neg_gamma;
            }
            else{
               switch2_gamma = (1 - etta_switch2)*switch2_gamma + etta_switch2*(abs(tau_high) - abs(tau_low));
            }

            alpha_switch2 = tanh(switch2_delta*(switch2_gamma - switch2_p))/2 + 0.5;
            float out_adm = 0;
            float out_imp = 0; 

            if(1){
                out_imp = Impedance_Controller(hw,ref[0]);
            }
            if(1){
                
                out_adm = Admitance_Controller(hw,ref[0]);
            }

            out = lowPass->process(out_imp*alpha_switch2 + out_adm*(1 - alpha_switch2), hw->get_dt());

        }
        
        else{
         /* POSITION LOOP */
                out = Impedance_Controller(hw,ref[0] + switch4_new_theta_ref);

                out = lowPass->process(out, hw->get_dt());    
                switch4_new_theta_ref = admittanceTF->process((-tau + tau_ref),hw->get_dt());
        }
    }

    
    *(hw->fric2) = out;

    return (out);
}