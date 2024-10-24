#include <forecast/controllers/CompliantHelioFL.hpp>

using namespace forecast;

CompliantHelioFL::CompliantHelioFL(float kp,float kd,float ki,float Kvc,float Kpc, float B_int, 
float leak_fix, float limit, float lambda,float gain_dob,float limit_dob, float gain_vc, float vc_limit, float start_x, float fl,
float gain_out, float filter_out, float dob_formulation, float pressure_predict, float Ml, float Kl, float Kdes, float Bdes,float Mdes,float K1, float K2, float massa_total, float psi_compliant, float F_fric)
    : 
      kp(kp),
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
    lowPassD_Xhat = utility::AnalogFilter::getLowPassFilterHz(freq);
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

float CompliantHelioFL::ForceController(const IHardware *hw, float ref){
    
    float tau = hw->get_tau_s(0) - once_force_imp;
    
    float deriv_force_desejada = (2.45*ref - 6*prev_ref_1 + 7.5*prev_ref_2 - 6.66*prev_ref_3 
    + 3.75*prev_ref_4 - 1.2*prev_ref_5 + 0.16*prev_ref_6)/
    (hw->get_dt());



    deriv_force_desejada = (ref - prev_ref_1)/hw->get_dt();

    deriv_force_desejada = lowPassx->process(deriv_force_desejada,hw->get_dt());

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

    float deriv_force = hw->get_d_tau_s(0);

    Pa = hw->get_pressure(3)*100000;
    Pb = hw->get_pressure(2)*100000;

    //Pt = hw->get_pressure(3)*100000;
    Ps = 16000000;
    Pt = 0; // Sensor de pressão com problema

    /*if(pressure_predict == 1){
        float De2 = pow(De, 2);
        float Dh2 = pow(Dh, 2);
        Aa = (M_PI*(De2))/4;
        Ab = ((M_PI*(De2))/4) - ((M_PI*(Dh2))/4);
        Ap = Aa;                    
        alfa = Ab/Aa;

        float Fco = 15;
        float Fso = 15;
        float Cs = 0.15;

        float fric = 0;
        float a = 0.001;


        if(abs(dx) < 0.1)
            fric = dx*30/0.1;
        else 
            fric = abs(dx)*(Fco + Fso*(exp((-abs(dx)/Cs))) + B_int*dx);


        Ps = 10000000;
        Pt = 0;
        Pa = -((tau - fric)/Aa)*(1/(1 - (1/alfa)));
        Pb = -((tau - fric)/Aa)*((1/(alfa*alfa))/(1 - (1/alfa)));
    }*/

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
        
        Va = Vpl + Aa*((x));
        Vb = Vpl + (L_cyl - (x))*Ab;

        if(ixv >= 0.00000f){
            g = Be*Aa*Kv*(round((Ps-Pa)/abs(Ps-Pa))*sqrt(abs(Ps-Pa))/Va + alfa*round((Pb-Pt)/abs(Pb-Pt))*sqrt(abs(Pb-Pt))/Vb);
            }
            
        else{
            g = Be*Aa*Kv*(round((Pa-Pt)/abs(Pa-Pt))*sqrt(abs(Pa-Pt))/Va + alfa*round((Ps-Pb)/abs(Ps-Pb))*sqrt(abs(Ps-Pb))/Vb);
            }

        //g = Be*Aa*Kv*( round((Pa-Pt)/abs(Pa-Pt))*sqrt(abs(Pa-Pt))/Va + alfa*round((Ps-Pb)/abs(Ps-Pb))*sqrt(abs(Ps-Pb))/Vb );

        f = Be*pow(Aa,2)*(pow(alfa,2)/Vb + 1/Va)*dx;

        dz = -lambda*Z - (lambda/((g)*Kpc))*((lambda*tau)/(Kpc*(g))- f*Kvc + (g)*Kpc*(last_out/1000));

        Z = Z + dz*hw->get_dt();

        disturb = Z + lambda*tau/(Kpc*(g));
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

    //expected_force = expected_force + deriv_force*hw->get_dt();
    //expected_force = tau;



    //*(hw->var10) = Ap*(Pa - alfa*Pb);
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
    *(hw->var5) = x_hat;
    *(hw->var6) = dx_hat;
    *(hw->var7) = ddx_hat;
    *(hw->var8) = tau;

    last_out = out;

    return out*gain_out;
}

float CompliantHelioFL::process(const IHardware *hw, std::vector<float> ref)
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