#include <forecast/controllers/Adaptative.hpp>

using namespace forecast;

Adaptative::Adaptative(float kp, float learn_rate,float learn_rate_h, float lear_rate_ap, float gain_out, float limit, float start_h, float start_disturb, float start_ap)
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
      learn_rate(learn_rate),
      learn_rate_ap(lear_rate_ap),
      learn_rate_h(learn_rate_h),
      kp(kp),
      gain_out(gain_out),
      limit(limit),
      hat_ap(start_ap),
      hat_h(start_h),
      hat_disturb(start_disturb)

{
    float freq = 40.0;
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
    In = 0.05f; //  Nominal valve input for Moog 24 [A]
    pn = 70.0E+5f; // Nominal pressure drop for Moog 24 [Pa]
    qn = 0.000166666f; // Nominal flow for Moog 24 [m^3/s]
    

    logs.push_back(&reference);
    
}

float Adaptative::process(const IHardware *hw, std::vector<float> ref)
{
    //Kvc = Kvc*0.089;
    //Kpc = Kpc*0.089;
    reference = ref[0];
    
    tau = hw->get_tau_s(1);
    dtau = hw->get_d_tau_s(1);

    x = hw->get_theta(1);
    dx = hw->get_d_theta(1);

    if (once == 1)
    {
        offset_x = x;
        once = 0;
    }

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

    ixv = last_out;
    //ixv = last_out;
    //Corrigir a leitura da corrente para checar qual equação de g utilizar

    err = ref[0] - tau;
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

    deriv_force_desejada = (ref[0] - prev_ref_1)/
    (hw->get_dt());

    deriv_force_desejada = lowPassD->process(deriv_force_desejada, hw->get_dt());

    prev_ref_6 = prev_ref_5;
    prev_ref_5 = prev_ref_4;
    prev_ref_4 = prev_ref_3;
    prev_ref_3 = prev_ref_2;
    prev_ref_2 = prev_ref_1;
    prev_ref_1 = ref[0];

    hat_h = hat_h + d_h*hw->get_dt();
    hat_disturb = hat_disturb + d_disturb*hw->get_dt();
    hat_ap = hat_ap + d_ap*hw->get_dt();

    out = (deriv_force_desejada*hat_h*1000)/g - kp*1000*(tau - reference)/g - 1000*hat_disturb + (1000*hat_ap*(-f))/g;

    //out = 0;

    d_h = -(learn_rate_h*(hat_h/abs(hat_h))*(tau - reference)*deriv_force_desejada)/g;
    d_ap = -((learn_rate_ap*(hat_h/abs(hat_h))*(tau - reference))*(-f))/g;
    d_disturb = -learn_rate*(hat_h/abs(hat_h))*(tau - reference)*(-1);

    if(out > limit){
        out = limit;
    }
    else if(out < -limit){
        out = -limit;
    }

    *(hw->var1) = out;
    *(hw->var2) = hat_disturb;
    *(hw->var3) = ref[0];
    *(hw->var4) = hat_ap;
    *(hw->var5) = hat_h;


    last_out = out;

    return out*gain_out;
}