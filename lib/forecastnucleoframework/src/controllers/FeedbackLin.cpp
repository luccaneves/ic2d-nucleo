#include <forecast/controllers/FeedbackLin.hpp>

using namespace forecast;

FeedbackLin::FeedbackLin(float kp,float kd,float ki,float Kvc,float Kpc, float B_int, 
float leak_fix, float limit, float lambda,float gain_dob,float limit_dob, float gain_vc, float vc_limit, float start_x, float fl,
float gain_out, float filter_out, float dob_formulation, float pressure_predict, float Ml, float Kl)
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
      Kl(Kl)
{
    float freq = 15.0;
    lowPass = utility::AnalogFilter::getLowPassFilterHz(freq);
    lowPassD = utility::AnalogFilter::getLowPassFilterHz(freq);
    lowPassx = utility::AnalogFilter::getLowPassFilterHz(freq);
    lowPassDx = utility::AnalogFilter::getLowPassFilterHz(freq);
    lowPassPa = utility::AnalogFilter::getLowPassFilterHz(freq);
    lowPassPb = utility::AnalogFilter::getLowPassFilterHz(freq);
    lowPassPs = utility::AnalogFilter::getLowPassFilterHz(freq);
    lowPassPt = utility::AnalogFilter::getLowPassFilterHz(freq);
    
    Be = 1.31E+9f; // Bulk modulus [Pa]
    De = 0.016f;  // Piston diameter [m]
    Dh = 0.01f; //  Rod diameter [m]
    L_cyl = 0.08f; // Stroke [m]
    //L_cyl = 0.32f; // Stroke [m]
    Vpl = 1.21E-3f; // Volume Pipeline [m^3]
    In = 0.01f; //  Nominal valve input for Moog 24 [A]
    pn = 70.0E+5f; // Nominal pressure drop for Moog 24 [Pa]
    qn = 0.000125f; // Nominal flow for Moog 24 [m^3/s]
    

    logs.push_back(&reference);
    
}

float FeedbackLin::process(const IHardware *hw, std::vector<float> ref)
{

    double Kth = 0;

    double Kce = 0;

    double Kqe = 0;

    double inv_model_num[6] = { 0   ,1.592230835850342 , -1.582310443952793 ,0,0 , 0};

    double inv_model_den[6] = {1.000000000000000  ,-1.984087638487695  , 0.984127320055285, 0   ,0,0};


    double filter_num[6] = {0.0  , 1.0000000000000000 , 0.0 ,0.0,0.0,0.0};
    double filter_den[6] = {1.000000000000000  , 0.0 , 0.0 ,0.0,0.0,0.0};

    //Kvc = Kvc*0.089;
    //Kpc = Kpc*0.089;
    reference = ref[0];
    
    tau = hw->get_tau_s(1);
    dtau = hw->get_d_tau_s(1);

    x = hw->get_theta(0);
    dx = hw->get_d_theta(0);
    ddx = hw->get_dd_theta(0);

    float deriv_force_desejada = (2.45*ref[0] - 6*prev_ref_1 + 7.5*prev_ref_2 - 6.66*prev_ref_3 
    + 3.75*prev_ref_4 - 1.2*prev_ref_5 + 0.16*prev_ref_6)/
    (hw->get_dt());

    prev_ref_6 = prev_ref_5;
    prev_ref_5 = prev_ref_4;
    prev_ref_4 = prev_ref_3;
    prev_ref_3 = prev_ref_2;
    prev_ref_2 = prev_ref_1;
    prev_ref_1 = ref[0];

    if (once == 1)
    {
        offset_x = x;
        once = 0;
    }

    float deriv_force = hw->get_d_tau_s(1);
    Pa = lowPassPa->process(hw->get_pressure(0)*100000, hw->get_dt());
    Pb = lowPassPb->process(hw->get_pressure(1)*100000, hw->get_dt());
    Ps = lowPassPs->process(hw->get_pressure(2)*100000, hw->get_dt());
    Pt = lowPassPt->process(hw->get_pressure(3)*100000, hw->get_dt());

    Pa = hw->get_pressure(0)*100000;
    Pb = hw->get_pressure(1)*100000;
    Ps = hw->get_pressure(2)*100000;
    //Pt = hw->get_pressure(3)*100000;
    Pt = 0; // Sensor de pressão com problema

    if(pressure_predict == 1){
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
    }

    if(Pa == Ps){
        Pa = Ps*0.99;
    }

    if(Pb == Ps){
        Pb = Ps*0.99;
    }


    ixv = hw->get_tau_m(0) - 0.0250*0;
    //ixv = last_out;
    //Corrigir a leitura da corrente para checar qual equação de g utilizar

    err = ref[0] - tau;
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

        d_disturb = (lambda/((g)*Kpc))*(deriv_force + f*Kvc - (g/1000)*hw->get_tau_m(0)*Kpc + B_int*hw->get_dd_theta(0) - (g)*disturb*Kpc);

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

        dz = -lambda*z - (lambda/((g)*Kpc))*((lambda*tau)/(Kpc*(g))- f*Kvc + (g)*Kpc*(hw->get_tau_m(0)/1000));

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

        d_disturb1 = (lambda/((h1)))*(deriv_force + f*Kvc - (g/1000)*hw->get_tau_m(0)*Kpc - h1*disturb1 - h2*disturb2 - (g)*disturb3*Kpc);

        //float dz = -lambda*z - (lambda/((g)*Kpc))*(lambda*tau- f*Kvc + (g)*Kpc*(hw->get_tau_m(0)/1000));

        disturb1 = disturb1 + d_disturb1*(hw->get_dt());

        d_disturb2 = (lambda/((h2)))*(deriv_force + f*Kvc - (g/1000)*hw->get_tau_m(0)*Kpc - h1*disturb1 - h2*disturb2 - (g)*disturb3*Kpc);

        //float dz = -lambda*z - (lambda/((g)*Kpc))*(lambda*tau- f*Kvc + (g)*Kpc*(hw->get_tau_m(0)/1000));

        disturb2 = disturb2 + d_disturb2*(hw->get_dt());

        d_disturb3 = (lambda/((g)*Kpc))*(deriv_force + f*Kvc - (g/1000)*hw->get_tau_m(0)*Kpc - h1*disturb1 - h2*disturb2 - (g)*disturb3*Kpc);

        //float dz = -lambda*z - (lambda/((g)*Kpc))*(lambda*tau- f*Kvc + (g)*Kpc*(hw->get_tau_m(0)/1000));

        disturb3 = disturb3 + d_disturb3*(hw->get_dt());

        disturb = disturb3 + ((disturb1*h1)/(g*Kpc)) + ((disturb2*h2)/(g*Kpc));

    }
    else if(dob_formulation == 3){

        //filter_num[6] = {0  , 1  , 0 ,0,0,0};
        //filter_den[6] = {1.000000000000000  , 0 , 0 ,0,0,0};

        Pt = 0;
        Ps = 10000000;

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

        f = Be*pow(Aa,2)*(pow(alfa,2)/Vb + 1/Va)*dx;

        float d_disturb1;
        float d_disturb2;

        d_disturb1 = lambda*(tau - Ml*ddx - Kl*(x - offset_x) - disturb1);

        d_disturb2 =  lambda*(deriv_force + f*Kvc - (g/1000)*hw->get_tau_m(0)*Kpc - disturb2);

        //Passar filtro nas derivadas dos disturbios

        disturb1 = disturb1 + d_disturb1*(hw->get_dt());

        disturb2 = disturb2 + d_disturb2*(hw->get_dt());

        disturb = (-d_disturb1 + disturb2)/(Kpc*g);

    }
    else if(dob_formulation == 4){
        
        //filter_num[6] = {0  , 1  , 0 ,0,0,0};
        //filter_den[6] = {1.000000000000000  , 0 , 0 ,0,0,0};

        Pt = 0;
        Ps = 10000000;

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

        f = Be*pow(Aa,2)*(pow(alfa,2)/Vb + 1/Va)*dx;

        float d_disturb1;
        float d_disturb2;

        disturb1 = lambda*(tau - Ml*ddx - Kl*(x - offset_x));

        disturb2 =  lambda*(deriv_force + f*Kvc - (g/1000)*hw->get_tau_m(0)*Kpc);

        //Passar filtro nas derivadas dos disturbios

        double output_filter_d1 = (disturb1*filter_num[0] + 
        prev1_dF*filter_num[1] + 
        prev2_dF*filter_num[2] + 
        prev3_dF*filter_num[3] + 
        prev4_dF*filter_num[4] - 
        filter_den[1]*prev1_filter_exit_dF -
        filter_den[2]*prev2_filter_exit_dF -
        filter_den[3]*prev3_filter_exit_dF -
        filter_den[4]*prev4_filter_exit_dF)/filter_den[0];

        double output_filter_d2 = (disturb2*filter_num[0] + 
        prev1_dx*filter_num[1] + 
        prev2_dx*filter_num[2] + 
        prev3_dx*filter_num[3] + 
        prev4_dx*filter_num[4] - 
        filter_den[1]*prev1_filter_exit_dx -
        filter_den[2]*prev2_filter_exit_dx -
        filter_den[3]*prev3_filter_exit_dx -
        filter_den[4]*prev4_filter_exit_dx)/filter_den[0];

        prev4_dF = prev3_dF;
        prev3_dF = prev2_dF;
        prev2_dF = prev1_dF;
        prev1_dF = d_disturb1;

        prev4_filter_exit_dF = prev3_filter_exit_dF;
        prev3_filter_exit_dF = prev2_filter_exit_dF;
        prev2_filter_exit_dF = prev1_filter_exit_dF;
        prev1_filter_exit_dF = output_filter_d1;

        prev4_dx = prev3_dx;
        prev3_dx = prev2_dx;
        prev2_dx = prev1_dx;
        prev1_dx = d_disturb2;

        prev4_filter_exit_dx = prev3_filter_exit_dx;
        prev3_filter_exit_dx = prev2_filter_exit_dx;
        prev2_filter_exit_dx = prev1_filter_exit_dx;
        prev1_filter_exit_dx = output_filter_d2;

        d_disturb1 = (2.45*output_filter_d1 - 6*prev1_disturb1 + 7.5*prev2_disturb1 - 6.66*prev3_disturb1 
        + 3.75*prev4_disturb1 - 1.2*prev5_disturb1 + 0.16*prev6_disturb1)/
        (hw->get_dt());

        prev6_disturb1 = prev5_disturb1;
        prev5_disturb1 = prev4_disturb1;
        prev4_disturb1 = prev3_disturb1;
        prev3_disturb1 = prev2_disturb1;
        prev1_disturb1 = output_filter_d1;

        disturb = (-d_disturb1 + output_filter_d2)/(Kpc*g);
    }
    else{
        Pt = 0;
        Ps = 10000000;

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

        double va0 = Vpl + L_cyl*Ap;
        double vb0 = Vpl + L_cyl*Ap*alfa;
        double uv0 = 0;
        double pa0 = alfa*Ps/(1 + alfa);
        double pb0 = Ps/(1 + alfa);


        Kth = Be*Ap*(1/va0 + alfa/vb0);

        double Kqa_pos = (Kv)*sqrt(Ps-pa0);
        double Kqb_pos= (Kv)*sqrt(pb0-Pt);

        double Kca_pos = Kv*uv0/(2*sqrt(Ps-pa0));
        double Kcb_pos = -Kv*uv0/(2*sqrt(pb0 - Pt));

        double Kce_pos = (1/(1 + alfa*alfa*alfa))*((vb0*Kca_pos - alfa*alfa*alfa*va0*Kcb_pos)/(vb0 + alfa*va0));

        double Kqe_pos = (vb0*Kqb_pos + alfa*va0*Kqb_pos)/(vb0 + alfa*va0);

        double Kqa_neg = (Kv)*sqrt(pa0 - Pt);
        double Kqb_neg= (Kv)*sqrt(Ps - pb0);

        double Kca_neg = -Kv*uv0/(2*sqrt(pa0 - Pt));
        double Kcb_neg = Kv*uv0/(2*sqrt(Ps - pb0));

        double Kce_neg = (1/(1 + alfa*alfa*alfa))*((vb0*Kca_neg - alfa*alfa*alfa*va0*Kcb_neg)/(vb0 + alfa*va0));

        double Kqe_neg = (vb0*Kqb_neg + alfa*va0*Kqb_neg)/(vb0 + alfa*va0);

        //if(ixv >=0){
            Kce = Kce_pos;
            Kqe = Kqe_pos;
        //}
        /*else{
            Kce = Kce_neg;
            Kqe = Kqe_neg;
        }*/


        double output_filter_dF = (dtau*filter_num[0] + 
        prev1_dF*filter_num[1] + 
        prev2_dF*filter_num[2] + 
        prev3_dF*filter_num[3] + 
        prev4_dF*filter_num[4] - 
        filter_den[1]*prev1_filter_exit_dF -
        filter_den[2]*prev2_filter_exit_dF -
        filter_den[3]*prev3_filter_exit_dF -
        filter_den[4]*prev4_filter_exit_dF)/filter_den[0];

        double output_filter_dx = (dx*filter_num[0] + 
        prev1_dx*filter_num[1] + 
        prev2_dx*filter_num[2] + 
        prev3_dx*filter_num[3] + 
        prev4_dx*filter_num[4] - 
        filter_den[1]*prev1_filter_exit_dx -
        filter_den[2]*prev2_filter_exit_dx -
        filter_den[3]*prev3_filter_exit_dx -
        filter_den[4]*prev4_filter_exit_dx)/filter_den[0];

        double output_filter_current = ((ixv/1000)*filter_num[0] + 
        prev1_current*filter_num[1] + 
        prev2_current*filter_num[2] + 
        prev3_current*filter_num[3] + 
        prev4_current*filter_num[4] - 
        filter_den[1]*prev1_filter_exit_current -
        filter_den[2]*prev2_filter_exit_current -
        filter_den[3]*prev3_filter_exit_current -
        filter_den[4]*prev4_filter_exit_current)/filter_den[0];


        prev4_current = prev3_current;
        prev3_current = prev2_current;
        prev2_current = prev1_current;
        prev1_current = (ixv/1000);

        prev4_filter_exit_current = prev3_filter_exit_current;
        prev3_filter_exit_current = prev2_filter_exit_current;
        prev2_filter_exit_current = prev1_filter_exit_current;
        prev1_filter_exit_current = output_filter_current;

        prev4_dF = prev3_dF;
        prev3_dF = prev2_dF;
        prev2_dF = prev1_dF;
        prev1_dF = dtau;

        prev4_filter_exit_dF = prev3_filter_exit_dF;
        prev3_filter_exit_dF = prev2_filter_exit_dF;
        prev2_filter_exit_dF = prev1_filter_exit_dF;
        prev1_filter_exit_dF = output_filter_dF;

        prev4_dx = prev3_dx;
        prev3_dx = prev2_dx;
        prev2_dx = prev1_dx;
        prev1_dx = dx;

        prev4_filter_exit_dx = prev3_filter_exit_dx;
        prev3_filter_exit_dx = prev2_filter_exit_dx;
        prev2_filter_exit_dx = prev1_filter_exit_dx;
        prev1_filter_exit_dx = output_filter_dx;

        disturb = (output_filter_dF/Kth + Ap*output_filter_dx*Kvc)/(Kqe*Kpc) - output_filter_current;
    }



    v = /*ref[0] +*/ kp * err + kd * derr + ki * ierr + deriv_force_desejada; //Sinal trocado, derivada da ref

    disturb = gain_dob*disturb;

    if(disturb > limit_dob/1000){
        disturb = limit_dob/1000;
    }
    else if(disturb < -limit_dob/1000){
        disturb = -limit_dob/1000;
    }

    if(fl == 1){
        out = (1000*(v))/(g*Kpc) + (f*Kvc*1000)/(g*Kpc) + B_int*hw->get_dd_theta(0)*1000/(g*Kpc) - disturb*1000 + leak_fix;
    }
    else{
        out = v - disturb*1000 + leak_fix + gain_vc*dx;
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
        d_force = -f*Kvc + (g*Kpc*(hw->get_tau_m(0)/1000 + disturb)) - B_int*hw->get_dd_theta(0);
    }
    else{
        d_expected_force = (((ixv)/1000 + disturb)*Kqe*Kpc - dx*Ap*Kvc)*Kth;

        if(hw->get_current_time() > 0.1){
            if(once_force == 1){
                once_force = 0;
                expected_force = tau;
            }
            expected_force = expected_force + d_expected_force*hw->get_dt(); 
        }
    }
    //expected_force = expected_force + deriv_force*hw->get_dt();
    //expected_force = tau;

    *(hw->fric1) = disturb*1000;
    *(hw->fric2) = (f*Kvc);
    *(hw->control_signal_teste) = (g*Kpc);
    *(hw->sprint_start_force) = expected_force;

    *(hw->var1) = out;
    *(hw->var2) = Ap*(Pa - alfa*Pb);
    *(hw->var3) = (Pa - alfa*Pb);
    *(hw->var4) = err;
    *(hw->var8) = disturb*1000;
    *(hw->var9) = expected_force;
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

    last_out = out;

    return out*gain_out;
}