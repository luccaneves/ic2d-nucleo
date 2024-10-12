#include <forecast/controllers/CompliantHelio.hpp>

using namespace forecast;

CompliantHelio::CompliantHelio(float max_f, float min_f, float max_g, float min_g, float eta, float psi, float limit, float gain_out, float gain_dob, float limit_dob, float lambda
,float max_disturb_current, float min_disturb_current, float disturb_model_gain, float kp, float ki, float kd, float Kdes, float Bdes,float Mdes
,float K1, float K2, float massa_total, float psi_compliant, float F_fric)
    : 
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
      limit(limit),
      gain_out(gain_out),
      max_f(max_f),
      max_g(max_g),
      min_f(min_f),
      min_g(min_g),
      psi(psi),
      etta(eta),
      gain_dob(gain_dob),
      limit_dob(limit_dob),
      lambda(lambda),
      max_disturb_current(max_disturb_current),
      min_disturb_current(min_disturb_current),
      disturb_model_gain(disturb_model_gain),
      kp(kp),
      ki(ki),
      kd(kd),
      Kdes(Kdes),
      Bdes(Bdes),
      Mdes(Mdes),
      K1(K1),
      K2(K2),
      massa_total(massa_total),
      psi_compliant(psi_compliant),
      F_fric(F_fric)
{
    float freq = 20.0;
    lowPass = utility::AnalogFilter::getLowPassFilterHz(freq);
    lowPassD = utility::AnalogFilter::getLowPassFilterHz(freq);
    lowPassx = utility::AnalogFilter::getLowPassFilterHz(freq);
    lowPassDx = utility::AnalogFilter::getLowPassFilterHz(freq);
    lowPassPa = utility::AnalogFilter::getLowPassFilterHz(freq);
    lowPassPb = utility::AnalogFilter::getLowPassFilterHz(freq);
    lowPassPs = utility::AnalogFilter::getLowPassFilterHz(freq);
    lowPassPt = utility::AnalogFilter::getLowPassFilterHz(freq);
    lowPassD_ErroImp = utility::AnalogFilter::getLowPassFilterHz(freq);

    lowPassD_z = utility::AnalogFilter::getLowPassFilterHz(freq);
    lowPassDD_Xhat = utility::AnalogFilter::getLowPassFilterHz(freq);
    lowPassDD_Xhat = utility::AnalogFilter::getLowPassFilterHz(freq);
    lowPassd_new_forca_desejada = utility::AnalogFilter::getLowPassFilterHz(freq);
    
    Be = 1.31E+9f; // Bulk modulus [Pa]
    De = 0.016f;  // Piston diameter [m]
    Dh = 0.01f; //  Rod diameter [m]
    L_cyl = 0.08f; // Stroke [m]
    //L_cyl = 0.32f; // Stroke [m]
    Vpl = 1.21E-3f; // Volume Pipeline [m^3]
    In = 0.05f; //  Nominal valve input for Moog 24 [A]
    pn = 70.0E+5f; // Nominal pressure drop for Moog 24 [Pa]
    qn = 0.0001666f; // Nominal flow for Moog 24 [m^3/s]
    

    if(Mdes > 0){
        double a_ADM[3] = {Mdes,Bdes,Kdes};
        double b_ADM[3] = {0.0,0.0,1.0};   
        transferFunction = new utility::AnalogFilter(2, a_ADM, b_ADM);
    }

    else {
        double a_ADM[2] = {Bdes,Kdes};
        double b_ADM[2] = {0.0,1.0};   
        transferFunction = new utility::AnalogFilter(1, a_ADM, b_ADM); 
    }


    logs.push_back(&reference);
    
}

float CompliantHelio::ForceController(const IHardware *hw, float ref){
    float tau = hw->get_tau_s(0) - once_force_imp;

    float deriv_force_desejada = (2.45*ref - 6*prev_ref_1 + 7.5*prev_ref_2 - 6.66*prev_ref_3 
    + 3.75*prev_ref_4 - 1.2*prev_ref_5 + 0.16*prev_ref_6)/
    (hw->get_dt());

    prev_ref_6 = prev_ref_5;
    prev_ref_5 = prev_ref_4;
    prev_ref_4 = prev_ref_3;
    prev_ref_3 = prev_ref_2;
    prev_ref_2 = prev_ref_1;
    prev_ref_1 = ref;

    float deriv_force = hw->get_d_tau_s(0);

    Pa = hw->get_pressure(3)*100000;
    Pb = hw->get_pressure(2)*100000;
    Ps = 16000000;
    Pt = 0;
    Pt = 0; // Sensor de pressão com problema

    if(Pa == Ps){
        Pa = Ps*0.99;
    }

    if(Pb == Ps){
        Pb = Ps*0.99;
    }

    ixv = last_out - 0.0250*0;
    //ixv = last_out;
    //Corrigir a leitura da corrente para checar qual equação de g utilizar

    err = ref - tau;
    derr = (err - errPast) / hw->get_dt();

    derr = (2.45*err - 6*prev_erro_1 + 7.5*prev_erro_2 - 6.66*prev_erro_3 
    + 3.75*prev_erro_4 - 1.2*prev_erro_5 + 0.16*prev_erro_6)/
    (hw->get_dt());

    derr = lowPass->process(derr,hw->get_dt());

    prev_erro_7 = prev_erro_6;
    prev_erro_6 = prev_erro_5;
    prev_erro_5 = prev_erro_4;
    prev_erro_4 = prev_erro_3;
    prev_erro_3 = prev_erro_2;
    prev_erro_2 = prev_erro_1;
    prev_erro_1 = err;

    float dist_gain_max = max_g*max_disturb_current;
    float dist_gain_min = min_g*min_disturb_current;

    float dist_gain_med = dist_gain_max/2 + dist_gain_min/2;



    ierr += err * hw->get_dt();
    errPast = err;

    float De2 = pow(De, 2);
    float Dh2 = pow(Dh, 2);

    Aa = (M_PI*(De2))/4;
    Ab = ((M_PI*(De2))/4) - ((M_PI*(Dh2))/4);
    Ap = Aa;                    
    alfa = Ab/Aa;
    Kv = qn/(In*sqrt(pn/2));

    Va = Vpl + Aa*(x);
    Vb = Vpl + (L_cyl - x)*Ab;

    if(ixv >= 0.00000f){
        g = Be*Aa*Kv*(round((Ps-Pa)/abs(Ps-Pa))*sqrt(abs(Ps-Pa))/Va + alfa*round((Pb-Pt)/abs(Pb-Pt))*sqrt(abs(Pb-Pt))/Vb);
        }
        
    else{
        g = Be*Aa*Kv*(round((Pa-Pt)/abs(Pa-Pt))*sqrt(abs(Pa-Pt))/Va + alfa*round((Ps-Pb)/abs(Ps-Pb))*sqrt(abs(Ps-Pb))/Vb);
        }

    //g = Be*Aa*Kv*( round((Pa-Pt)/abs(Pa-Pt))*sqrt(abs(Pa-Pt))/Va + alfa*round((Ps-Pb)/abs(Ps-Pb))*sqrt(abs(Ps-Pb))/Vb );


    f = -Be*pow(Aa,2)*(pow(alfa,2)/Vb + 1/Va)*dx;

    beta = sqrt(max_g/min_g);
    gain_g_med = sqrt(max_g*min_g);
    gain_f_med = (max_f + min_f)/2;

    deriv_force_desejada = (2.45*ref - 6*prev_ref_1 + 7.5*prev_ref_2 - 6.66*prev_ref_3 
    + 3.75*prev_ref_4 - 1.2*prev_ref_5 + 0.16*prev_ref_6)/
    (hw->get_dt());


    deriv_force_desejada = (ref - prev_ref_1)/ (hw->get_dt());

    deriv_force_desejada = lowPassD->process(deriv_force_desejada, hw->get_dt());

    prev_ref_6 = prev_ref_5;
    prev_ref_5 = prev_ref_4;
    prev_ref_4 = prev_ref_3;
    prev_ref_3 = prev_ref_2;
    prev_ref_2 = prev_ref_1;
    prev_ref_1 = ref;

    float u = (deriv_force_desejada - gain_f_med*f + disturb_model_gain*dist_gain_med*g + kp*(ref - tau) + ki*ierr + kd*derr);

    float k = (beta*(abs((max_f - gain_f_med)*(f)) + etta) + (beta - 1)*abs(u) + beta*(abs(dist_gain_max - dist_gain_med)*(g)));

    float sat_ = 0;
    float s = tau - ref;

    if(abs(s/psi) <= 1){
        sat_ = s/psi;
        }
    else{
        if(s >= 0){
           sat_ = 1; 
        }
        else if(s < 0){
            sat_ = -1; 
        }
    }
    

    //current = 1/(0.86*g)*(u - k*sign(s));
    out = ((u - k*sat_)*1000)/(gain_g_med*g);

    if(out > limit){
        out = limit;
    }
    else if(out < -limit){
        out = -limit;
    }

    //*(hw->var4) = out;
    //out = lowPass->process(out,hw->get_dt());

    last_out = out;

    *(hw->var1) = erro_imp;
    *(hw->var2) = deriv_erro_imp;
    *(hw->var3) = ref;
    *(hw->var4) = reference;
    *(hw->var5) = x_hat;
    *(hw->var6) = dx_hat;
    *(hw->var7) = ddx_hat;
    *(hw->var8) = tau;


    return out*gain_out;

}

float CompliantHelio::process(const IHardware *hw, std::vector<float> ref)
{
    float start_time = 1.5;
    //Kvc = Kvc*0.089;
    //Kpc = Kpc*0.089;
    reference = ref[0];
    
    tau = hw->get_tau_s(1) - once_force;
    dtau = hw->get_d_tau_s(1);

    x = hw->get_theta(1) - offset_x;
    dx = hw->get_d_theta(1);
    ddx = hw->get_dd_theta(1);

    if(once == 1 && hw->get_current_time() > start_time/2){
        once = 0;
        offset_x = x;
        once_force = hw->get_tau_s(1);
        once_force_imp = hw->get_tau_s(0);
    }

    
    if(hw->get_current_time() > start_time){
        x_hat = (transferFunction->process(tau,hw->get_dt()) + ref[0]);

        dx_hat = (x_hat - last_x_hat)/(hw->get_dt());

        dx_hat = lowPassD_Xhat->process(dx_hat,hw->get_dt());

        last_x_hat = x_hat;

        ddx_hat = (dx_hat - last_dx_hat)/hw->get_dt();

        ddx_hat = lowPassDD_Xhat->process(ddx_hat,hw->get_dt());

        last_dx_hat = dx_hat;






        z = x - x_hat;  

        dz = (z - last_z)/(hw->get_dt());

        dz = lowPassD_z->process(dz,hw->get_dt());

        last_z = z;



        float sat_ = 0;
        float s = dz;

        if(abs(s/psi_compliant) <= 1){
            sat_ = s/psi_compliant;
            }
        else{
            if(s >= 0){
            sat_ = 1; 
            }
            else if(s < 0){
                sat_ = -1; 
            }
        }


        new_forca_desejada = -K1*z - K2*dz - tau - F_fric*sat_ + massa_total*ddx_hat;


        erro_imp = x - ref[0];

        deriv_erro_imp = (erro_imp - last_erro_imp)/hw->get_dt();

        last_erro_imp = erro_imp;

        deriv_erro_imp = lowPassD_ErroImp->process(deriv_erro_imp,hw->get_dt());

        float tau_ref = - Kdes*(erro_imp) -  Bdes*deriv_erro_imp - Mdes*ddx;



        out = ForceController(hw,new_forca_desejada);
    }
    else{
        out = 0;
    }

    return out;
}