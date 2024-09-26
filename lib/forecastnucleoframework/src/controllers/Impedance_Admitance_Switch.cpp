#include <forecast/controllers/Impedance_Admitance_Switch.hpp>

using namespace forecast;

Impedance_Admitance_Switch::Impedance_Admitance_Switch(
float kp, float ki, float kd,
float Mdes, float Bdes, float Kdes, 
float gain_dob, 
float Kp_pos, float Kd_pos, float Ki_pos, 

float switch_method,
float duty_delta,float n_percent, 
float alpha_max,
float etta_switch2, float freq_cutoff_switch2, float switch2_neg_gamma, float switch2_threshold_force, float switch2_delta, float switch2_p
)
    : kp(kp),
      ki(ki),
      kd(kd),
      Kdes(Kdes),
      Bdes(Bdes),
      Mdes(Mdes),
      gain_dob(gain_dob),
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
    limit = 30;
    limit_dob = 30;
    lambda = 5;
    Kvc = 1;
    Kpc = 1.2;
    gain_out = 1;

    Be = 1.31E+9f; // Bulk modulus [Pa]
    De = 0.016f;  // Piston diameter [m]
    Dh = 0.01f; //  Rod diameter [m]
    L_cyl = 0.08f; // Stroke [m]
    //L_cyl = 0.32f; // Stroke [m]
    Vpl = 1.21E-3f; // Volume Pipeline [m^3]
    In = 0.05f; //  Nominal valve input for Moog 24 [A]
    pn = 70.0E+5f; // Nominal pressure drop for Moog 24 [Pa]
    qn = 0.0001666f; // Nominal flow for Moog 24 [m^3/s]




    logs.push_back(&reference);

    lowPass = utility::AnalogFilter::getLowPassFilterHz(20.0f);
    lowPassD = utility::AnalogFilter::getLowPassFilterHz(15.0f);
    
    lowPass_DerivRef = utility::AnalogFilter::getLowPassFilterHz(5); 
    lowPass_DerivRefForce = utility::AnalogFilter::getLowPassFilterHz(5);
    lowPass_DerivErroImp = utility::AnalogFilter::getLowPassFilterHz(5); 
    lowPass_FilterOut = utility::AnalogFilter::getLowPassFilterHz(5); 


    Switch2_LowPass = utility::AnalogFilter::getLowPassFilterHz(freq_cutoff_switch2);

    float value = 0.083; // 2hz de corte

    double den_highpass[2] = {value,1};
    double num_highpass[2] = {value,0};   

    Switch2_HighPass = new utility::AnalogFilter(1, den_highpass, num_highpass);

    if(Mdes > 0){
        double a_ADM[3] = {Mdes,Bdes,Kdes};
        double b_ADM[3] = {0.0,0.0,1.0};   
        admittanceTF = new utility::AnalogFilter(2, a_ADM, b_ADM);
    }

    else {
        double a_ADM[2] = {Bdes,Kdes};
        double b_ADM[2] = {0.0,1.0};   
        admittanceTF = new utility::AnalogFilter(1, a_ADM, b_ADM); 
    }
}

float Impedance_Admitance_Switch::ForceController(const IHardware *hw, float ref){

    float deriv_force_desejada = (2.45*ref - 6*prev_ref_1 + 7.5*prev_ref_2 - 6.66666*prev_ref_3 
    + 3.75*prev_ref_4 - 1.2*prev_ref_5 + 0.16666*prev_ref_6)/
    (hw->get_dt());

    deriv_force_desejada = (ref - prev_ref_1)/(hw->get_dt());

    deriv_force_desejada = lowPass_DerivRefForce->process(deriv_force_desejada,hw->get_dt());

    prev_ref_6 = prev_ref_5;
    prev_ref_5 = prev_ref_4;
    prev_ref_4 = prev_ref_3;
    prev_ref_3 = prev_ref_2;
    prev_ref_2 = prev_ref_1;
    prev_ref_1 = ref;

    float deriv_force = hw->get_d_tau_s(1);

    Pa = hw->get_pressure(3)*100000;
    Pb = hw->get_pressure(2)*100000;

    Ps = 16000000;
    Pt = 0; // Sensor de pressão com problema

    if(Pa == Ps){
        Pa = Ps*0.99;
    }

    if(Pb == Ps){
        Pb = Ps*0.99;
    }

    if(Pa == Pt){
        Pa = Ps*0.02;
    }

    if(Pb == Pt){
        Pb = Ps*0.02;
    }


    ixv = last_out - 0.0250*0;
    //ixv = last_out;
    //Corrigir a leitura da corrente para checar qual equação de g utilizar

    err = ref - tau;
    derr = (err - errPast) / hw->get_dt();

    derr = (2.45*err - 6*prev_erro_1 + 7.5*prev_erro_2 - 6.66*prev_erro_3 
    + 3.75*prev_erro_4 - 1.2*prev_erro_5 + 0.16*prev_erro_6)/
    (hw->get_dt());

    derr = lowPassD->process(derr,hw->get_dt());

    prev_erro_7 = prev_erro_6;
    prev_erro_6 = prev_erro_5;
    prev_erro_5 = prev_erro_4;
    prev_erro_4 = prev_erro_3;
    prev_erro_3 = prev_erro_2;
    prev_erro_2 = prev_erro_1;
    prev_erro_1 = err;

    ierr += err * hw->get_dt();
    errPast = err;

    float De2 = pow(De, 2);
    float Dh2 = pow(Dh, 2);

    B_int = 0;
    float d_disturb;

    float dz;


    
    if(1){
        Aa = (M_PI*(De2))/4;
        Ab = ((M_PI*(De2))/4) - ((M_PI*(Dh2))/4);
        Ap = Aa;                    
        alfa = Ab/Aa;
        Kv = qn/(In*sqrt(pn/2));
        
        Va = Vpl + Aa*((x - offset_x));
        Vb = Vpl + (L_cyl - (x - offset_x))*Ab;

        if(ixv >= 0.00000f){
            g = Be*Aa*Kv*(round((Ps-Pa)/abs(Ps-Pa))*sqrt(abs(Ps-Pa))/Va + alfa*round((Pb-Pt)/abs(Pb-Pt))*sqrt(abs(Pb-Pt))/Vb);
            }
            
        else{
            g = Be*Aa*Kv*(round((Pa-Pt)/abs(Pa-Pt))*sqrt(abs(Pa-Pt))/Va + alfa*round((Ps-Pb)/abs(Ps-Pb))*sqrt(abs(Ps-Pb))/Vb);
            }

        //g = Be*Aa*Kv*( round((Pa-Pt)/abs(Pa-Pt))*sqrt(abs(Pa-Pt))/Va + alfa*round((Ps-Pb)/abs(Ps-Pb))*sqrt(abs(Ps-Pb))/Vb );

        f = Be*pow(Aa,2)*(pow(alfa,2)/Vb + 1/Va)*dx;

        dz = -lambda*z - (lambda/((g)*Kpc))*((lambda*tau)/(Kpc*(g))- f*Kvc + (g)*Kpc*(last_out/1000));

        z = z + dz*hw->get_dt();

        disturb = z + lambda*tau/(Kpc*(g));
    }


    v = /*ref[0] +*/ kp * err + kd * derr + ki * ierr; //Sinal trocado, derivada da ref

    disturb = gain_dob*disturb;

    if(disturb > limit_dob/1000){
        disturb = limit_dob/1000;
    }
    else if(disturb < -limit_dob/1000){
        disturb = -limit_dob/1000;
    }

    out = (1000*(v + deriv_force_desejada))/(g*Kpc) + (f*Kvc*1000)/(g*Kpc) + B_int*hw->get_dd_theta(0)*1000/(g*Kpc) - disturb*1000 + leak_fix;

    if(out > limit){
        out = limit;
    }
    else if(out < -limit){
        out = -limit;
    }

    //*(hw->var4) = out;

    //Lucca: Adicionado filtro na saída. Vai dar merda?
    if(filter_out == 1){
        out = lowPass->process(out,hw->get_dt());
    }

    *(hw->var1) = d_force;
    *(hw->var2) = deriv_force;
    *(hw->var3) = out;
    *(hw->var4) = hw->get_tau_m(1);

    *(hw->var7) = deriv_ref;
    *(hw->var8) = disturb;
    *(hw->var9) = reference;

    last_out = hw->get_tau_m(1);

    return out*gain_out;

}

float Impedance_Admitance_Switch::PositionController(const IHardware *hw, float ref){
    err_adm = (ref + theta_ref) - x;
    derr_adm = (2.45*err_adm - 6*last_erro_1 + 7.5*last_erro_2 - 6.66*last_erro_3 + 3.75*last_erro_4 - 1.2*last_erro_5 + 0.16*last_erro_6)/(hw->get_dt());

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

float Impedance_Admitance_Switch::Impedance_Controller(const IHardware *hw, float ref){
    //Kvc = Kvc*0.089;
    //Kpc = Kpc*0.089;
    reference = ref;

    float tau_ref = Kdes*(erro_imp_adm) + Bdes*(deriv_erro_imp_adm);

    float out = ForceController(hw,tau_ref);

    return out;
}

float Impedance_Admitance_Switch::Admitance_Controller(const IHardware *hw, float ref){
    //Kvc = Kvc*0.089;
    //Kpc = Kpc*0.089;
    reference = ref;

    float out = PositionController(hw,ref_adm);

    return out;
}

float Impedance_Admitance_Switch::process(const IHardware *hw, std::vector<float> ref)
{
    tau = hw->get_tau_s(1) - once_force;
    dtau = hw->get_d_tau_s(1);
    x = hw->get_theta(1) - theta_eq;
    dx = hw->get_d_theta(1);
    ddx = hw->get_dd_theta(1);

    deriv_ref = (ref[0] - prev1_ref_x_1)/(hw->get_dt());

    reference = ref[0];

    deriv_ref = lowPass_DerivRef->process(deriv_ref,hw->get_dt());

    erro_imp_adm = reference - x;

    deriv_erro_imp_adm = (erro_imp_adm - last_erro_imp_adm)/(hw->get_dt());

    deriv_erro_imp_adm = lowPass_DerivErroImp->process(deriv_erro_imp_adm,hw->get_dt());

    int_erro_imp_adm = int_erro_imp_adm +  erro_imp_adm*hw->get_dt();

    last_erro_imp_adm = erro_imp_adm;



    prev1_ref_x_6 = prev1_ref_x_5;
    prev1_ref_x_5 = prev1_ref_x_4;
    prev1_ref_x_4 = prev1_ref_x_3;
    prev1_ref_x_3 = prev1_ref_x_2;
    prev1_ref_x_2 = prev1_ref_x_1;
    prev1_ref_x_1 = reference;


    reference = ref[0];
    /* Get the equilibrium state */
    if (once && hw->get_current_time() > 0)
    {
        tempo_start =  hw->get_t();
        theta_eq = x;
        once = false;
        once_force = tau;
        errPast = 0;
    }
    
    //Colocar código para definir se é impedancia ou admitancia


    theta_ref = admittanceTF->process(tau,hw->get_dt());

    ref_adm = (reference - theta_ref) - (x);

    if(hw->get_current_time() >= 0){
        if(switch_method == 0 || switch_method == 1){
            if(switch_method == 0){
                float time = hw->get_t(); //Tempo em s

                if(
                (
                (time_start_cycle) <= time) 
                && 
                (time <= (time_start_cycle + (1 - n_percent)*duty_delta)
                )
                )
                {
                    flag_impedance_admitance = 1;
                }

                else{
                    if (flag_impedance_admitance == 1){
                        ref_adm = (erro_imp_adm)*(1 - Kdes/Kp_pos) + deriv_erro_imp_adm*(-Bdes/Kp_pos) + int_erro_imp_adm*(Ki_pos/Kp_pos);
                    }
                    flag_impedance_admitance = 0;

                }

                if(time >= (time_start_cycle + duty_delta)){
                    time_start_cycle = time;
                }
            }

            else if(switch_method == 1){
                
                if(abs(ref_adm - x) > alpha_max){
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

            //out = lowPass->process(out,hw->get_dt());

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
                    float tau_ref = -Kdes*(x - reference) - Bdes*(dx - deriv_ref) - Mdes*ddx;
                    switch4_new_theta_ref = admittanceTF->process((-tau + tau_ref),hw->get_dt());
            }
        }
    }

    else{
        out = 0;
    }

    *(hw->var4) = deriv_erro_imp_adm;
    *(hw->var5) = x;
    *(hw->var6) = tau;
    *(hw->var7) = deriv_ref;
    *(hw->var8) = flag_impedance_admitance;
    *(hw->var9) = reference;


    out = lowPass_FilterOut->process(out,hw->get_dt());
    *(hw->fric2) = out;



    return (out);
}