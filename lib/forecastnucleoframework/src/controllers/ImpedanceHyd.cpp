#include <forecast/controllers/ImpedanceHyd.hpp>

using namespace forecast;

ImpedanceHyd::ImpedanceHyd(float kp,float kd,float ki,float Kvc,float Kpc, float B_int, 
float leak_fix, float limit, float lambda,float gain_dob,float limit_dob, float gain_vc, float vc_limit, float start_x, float fl,
float gain_out, float filter_out, float dob_formulation, float pressure_predict, float Ml, float Kl, float Kdes, float Bdes,
float Mdes, float sensor_select, float freq)
    : kp(kp),
      kd(kd),
      ki(ki),
      Kvc(Kvc),
      Kpc(Kpc),
      tau(0.0f),
      dtau(0.0f),
      errPast(0.f),
      err(0.f),
      derr(0.f),
      ierr(0.f),
      x(0.f),
      dx(0.f),
      Aa(0.0f),
      Ap(0.0f),
      Ab(0.0f),
      alfa(0.0f),
      Va(0.0f),
      Vb(0.0f),
      Kv(0.0f),
      Pt(0.0f),
      Ps(0.0f),
      Pa(0.0f),
      Pb(0.0f),
      f(0.0f),
      g(0.0f),
      v(0.0f),
      B_int(B_int),
      leak_fix(leak_fix),
      limit(limit),
      lambda(lambda),
      gain_dob(gain_dob),
      limit_dob(limit_dob),
      gain_vc(gain_vc),
      vc_limit(vc_limit),
      start_x(start_x),
      fl(fl),
      gain_out(gain_out),
      filter_out(filter_out),
      dob_formulation(dob_formulation),
      pressure_predict(pressure_predict),
      Ml(Ml),
      Kl(Kl),
      Kdes(Kdes),
      Bdes(Bdes),
      Mdes(Mdes),
      sensor_select(sensor_select),
      freq(freq)
{
    float freq_corte = freq;
    lowPass = utility::AnalogFilter::getLowPassFilterHz(freq_corte);
    lowPassD = utility::AnalogFilter::getLowPassFilterHz(freq_corte);
    lowPassx = utility::AnalogFilter::getLowPassFilterHz(freq_corte);
    lowPassDx = utility::AnalogFilter::getLowPassFilterHz(freq_corte);
    lowPassPa = utility::AnalogFilter::getLowPassFilterHz(freq_corte);
    lowPassPb = utility::AnalogFilter::getLowPassFilterHz(freq_corte);
    lowPassPs = utility::AnalogFilter::getLowPassFilterHz(freq_corte);
    lowPassPt = utility::AnalogFilter::getLowPassFilterHz(freq_corte);
    lowPassD_ErroImp = utility::AnalogFilter::getLowPassFilterHz(freq_corte);
    
    Be = 1.31E+9f; // Bulk modulus [Pa]
    De = 0.016f;  // Piston diameter [m]
    Dh = 0.01f; //  Rod diameter [m]
    L_cyl = 0.08f; // Stroke [m]
    //L_cyl = 0.32f; // Stroke [m]
    Vpl = 0.00121; // Volume Pipeline [m^3]
    In = 0.05f; //  Nominal valve input for Moog 24 [A]
    pn = 70.0E+5f; // Nominal pressure drop for Moog 24 [Pa]
    qn = 0.0001666f; // Nominal flow for Moog 24 [m^3/s]
    

    logs.push_back(&reference);
    
}

float ImpedanceHyd::ForceController(const IHardware *hw, float ref){
    float deriv_force_desejada = (2.45*ref - 6*prev_ref_1 + 7.5*prev_ref_2 - 6.66*prev_ref_3 
    + 3.75*prev_ref_4 - 1.2*prev_ref_5 + 0.16*prev_ref_6)/
    (hw->get_dt());



    deriv_force_desejada = (ref - prev_ref_1)/hw->get_dt();



    prev_ref_6 = prev_ref_5;
    prev_ref_5 = prev_ref_4;
    prev_ref_4 = prev_ref_3;
    prev_ref_3 = prev_ref_2;
    prev_ref_2 = prev_ref_1;
    prev_ref_1 = ref;

    if (once == 1)
    {
        offset_x = x;
        once = 0;
    }
    force_sensor_id = sensor_select;
    float deriv_force = hw->get_d_tau_s(force_sensor_id);

    Pa = lowPassPa->process(hw->get_pressure(2)*100000,hw->get_dt());
    Pb = lowPassPb->process(hw->get_pressure(3)*100000,hw->get_dt());

    //Pt = hw->get_pressure(3)*100000;
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

    float d_disturb1;
    float d_disturb2;
    float d_disturb3;

    float dz;

    float h1;

    float h2;

    
    if(dob_formulation == 0){
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

        d_disturb = (lambda/((g)*Kpc))*(deriv_force + f*Kvc - (g/1000)*last_out*Kpc + B_int*hw->get_dd_theta(0) - (g)*disturb*Kpc);

        //float dz = -lambda*z - (lambda/((g)*Kpc))*(lambda*tau- f*Kvc + (g)*Kpc*(hw->get_tau_m(0)/1000));

        disturb = disturb + d_disturb*(hw->get_dt());
    }
    else if(dob_formulation == 1){
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
    else if(dob_formulation == 2){
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

        h1 = Ap*Be*(Pb - Pa)*(1/Va + alfa/Vb);

        h2 =  Ap*Be*(1/Va + alfa/Vb)*(sqrt(Ps - Pa) - sqrt(Pa - Pt) + sqrt(Pb - Pt) - sqrt(Ps - Pb));

        d_disturb1 = (lambda/((h1)))*(deriv_force + f*Kvc - (g/1000)*last_out*Kpc - h1*disturb1 - h2*disturb2 - (g)*disturb3*Kpc);

        //float dz = -lambda*z - (lambda/((g)*Kpc))*(lambda*tau- f*Kvc + (g)*Kpc*(hw->get_tau_m(0)/1000));

        disturb1 = disturb1 + d_disturb1*(hw->get_dt());

        d_disturb2 = (lambda/((h2)))*(deriv_force + f*Kvc - (g/1000)*last_out*Kpc - h1*disturb1 - h2*disturb2 - (g)*disturb3*Kpc);

        //float dz = -lambda*z - (lambda/((g)*Kpc))*(lambda*tau- f*Kvc + (g)*Kpc*(hw->get_tau_m(0)/1000));

        disturb2 = disturb2 + d_disturb2*(hw->get_dt());

        d_disturb3 = (lambda/((g)*Kpc))*(deriv_force + f*Kvc - (g/1000)*last_out*Kpc - h1*disturb1 - h2*disturb2 - (g)*disturb3*Kpc);

        //float dz = -lambda*z - (lambda/((g)*Kpc))*(lambda*tau- f*Kvc + (g)*Kpc*(hw->get_tau_m(0)/1000));

        disturb3 = disturb3 + d_disturb3*(hw->get_dt());

        disturb = disturb3 + ((disturb1*h1)/(g*Kpc)) + ((disturb2*h2)/(g*Kpc));
    }




    v = /*ref[0] +*/ kp * err + kd * derr + ki * ierr; //Sinal trocado, derivada da ref

    disturb = gain_dob*disturb;

    if(disturb > limit_dob/1000){
        disturb = limit_dob/1000;
    }
    else if(disturb < -limit_dob/1000){
        disturb = -limit_dob/1000;
    }

    if(fl == 1){
        out = (1000*(v + deriv_force_desejada))/(g*Kpc) + (f*Kvc*1000)/(g*Kpc) + B_int*hw->get_dd_theta(0)*1000/(g*Kpc) - disturb*1000 + leak_fix;
    }
    else{
        out = v;
    }
    
    float d_expected_force = 0;
    //expected_force = tau;

    if(dob_formulation == 0 || dob_formulation == 1 || dob_formulation == 2 || dob_formulation == 3){
        if(hw->get_current_time() > 3){
            if(once_force == 1){
                once_force = 0;
                expected_force = tau;
            }
            expected_force = expected_force + d_force*hw->get_dt(); 
        }
        d_force = -f*Kvc + (g*Kpc*(last_out/1000 + disturb)) - B_int*hw->get_dd_theta(0);
    }

    out = out;

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

    *(hw->var1) = erro_imp;
    *(hw->var2) = deriv_erro_imp;
    *(hw->var3) = ref;
    *(hw->var4) = reference;
    *(hw->var5) = tau;

    last_out = out;

    return out*gain_out;

}

float ImpedanceHyd::process(const IHardware *hw, std::vector<float> ref)
{
    float start_time = 0;
    force_sensor_id = sensor_select;

    if(once == 1 && hw->get_current_time() > start_time/2){
        once = 0;
        offset_x = x;
        once_force = hw->get_tau_s(force_sensor_id);
    }

    if(hw->get_current_time() > start_time){

        //Kvc = Kvc*0.089;
        //Kpc = Kpc*0.089;
        reference = ref[0];
        
        tau = lowPass->process(hw->get_tau_s(force_sensor_id),hw->get_dt());
        dtau = lowPassD->process(hw->get_d_tau_s(force_sensor_id),hw->get_dt());

        x = hw->get_theta(1) - offset_x;
        dx = hw->get_d_theta(1);
        ddx = hw->get_dd_theta(1);

        erro_imp = x - ref[0];

        deriv_erro_imp = (erro_imp - last_erro_imp)/hw->get_dt();

        deriv_erro_imp = lowPassD_ErroImp->process(deriv_erro_imp,hw->get_dt());

        last_erro_imp = erro_imp;

        float tau_ref = - Kdes*(erro_imp) -  Bdes*deriv_erro_imp - Mdes*ddx;

        out = ForceController(hw,tau_ref);
    }
    else{
        out = 0;
    }

    return out;
}