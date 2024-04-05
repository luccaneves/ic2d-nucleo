#include <forecast/controllers/FeedbackLin.hpp>

using namespace forecast;

FeedbackLin::FeedbackLin(float kp,float kd,float ki,float Kvc,float Kpc, float B_int, 
float leak_fix, float limit, float lambda,float gain_dob,float limit_dob, float gain_vc, float vc_limit, float start_x)
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
      start_x(start_x)
{
    float freq = 5.0;
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

    //Kvc = Kvc*0.089;
    //Kpc = Kpc*0.089;
    reference = ref[0];
    
    tau = hw->get_tau_s(1);
    dtau = hw->get_d_tau_s(1);

    x = hw->get_theta(0);
    dx = hw->get_d_theta(0);

    if (once == 1)
    {
        offset_x = x;
        once = 0;
    }

    float deriv_force = hw->get_d_tau_s(1);
    //Pa = lowPassPa->process(hw->get_pressure(0)*100000, hw->get_dt());
    //Pb = lowPassPb->process(hw->get_pressure(1)*100000, hw->get_dt());
    //Ps = lowPassPs->process(hw->get_pressure(2)*100000, hw->get_dt());
    //Pt = lowPassPt->process(hw->get_pressure(3)*100000, hw->get_dt());


    Pa = lowPassPa->process(hw->get_pressure(0)*100000,hw->get_dt());
    Pb = lowPassPb->process(hw->get_pressure(1)*100000,hw->get_dt());
    Ps = lowPassPs->process(hw->get_pressure(2)*100000,hw->get_dt());
    Pt = lowPassPt->process(hw->get_pressure(3)*100000,hw->get_dt());


    ixv = hw->get_tau_m(0) - 0.0190*20;
    //Corrigir a leitura da corrente para checar qual equação de g utilizar

    err = ref[0] - tau;

    //derr = (err - errPast) / hw->get_dt();

    derr = (2.45*err - 6*prev_erro_1 + 7.5*prev_erro_2 - 6.66*prev_erro_3 
    + 3.75*prev_erro_4 - 1.2*prev_erro_5 + 0.16*prev_erro_6)/
    (hw->get_dt());

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
    
    Va = Vpl + Aa*(start_x + (x - offset_x));
    Vb = Vpl + (L_cyl - (start_x + (x - offset_x)))*Ab;

    if(ixv >= 0.00000f){
        g = Be*Aa*Kv*(round((Ps-Pa)/abs(Ps-Pa))*sqrt(abs(Ps-Pa))/Va + alfa*round((Pb-Pt)/abs(Pb-Pt))*sqrt(abs(Pb-Pt))/Vb);
        }
        
    else{
        g = Be*Aa*Kv*(round((Pa-Pt)/abs(Pa-Pt))*sqrt(abs(Pa-Pt))/Va + alfa*round((Ps-Pb)/abs(Ps-Pb))*sqrt(abs(Ps-Pb))/Vb);
        }

    //g = Be*Aa*Kv*( round((Pa-Pt)/abs(Pa-Pt))*sqrt(abs(Pa-Pt))/Va + alfa*round((Ps-Pb)/abs(Ps-Pb))*sqrt(abs(Ps-Pb))/Vb );


    f = Be*pow(Aa,2)*(pow(alfa,2)/Vb + 1/Va)*dx;


    float d_disturb = (lambda*1000/(g*Kpc))*(deriv_force + f*Kvc - (g/1000)*hw->get_tau_m(0)*Kpc + B_int*hw->get_dd_theta(0) - (g/1000)*disturb*Kpc);

    disturb = disturb + d_disturb*(hw->get_dt());

    disturb = lowPassD->process(disturb,hw->get_dt());

    //f = Be*pow(Aa,2)*( -pow(alfa,2)/Vb - 1/Va )*dx;
   
    v = /*ref[0] +*/ kp * err + kd * derr + ki * ierr;

    //out = Kpc/(g)*(-Kvc*f*1000 + v);

    disturb = gain_dob*disturb;

    if(disturb > limit_dob){
        disturb = limit_dob;
    }
    else if(disturb < -limit_dob){
        disturb = -limit_dob;
    }

    //TODO: Voltar B_int para o FL
    out = (1000*(v))/(g*Kpc) + (f*Kvc*1000)/(g*Kpc) + B_int*hw->get_dd_theta(0)*1000/(g*Kpc) - disturb;

    //TODO: Tirar isso depois plmrds
    //out = v - disturb;

    deriv_force = -f*Kvc + (g*Kpc*(hw->get_tau_m(0) + disturb))/1000 - B_int*hw->get_dd_theta(0); 

    if(hw->get_current_time() > 2){
       expected_force = expected_force + deriv_force*hw->get_dt(); 
    }
    //expected_force = expected_force + deriv_force*hw->get_dt();

    *(hw->fric1) = disturb;
    *(hw->fric2) = -f*Kvc;
    *(hw->control_signal_teste) = (g*Kpc*(hw->get_tau_m(0) + disturb))/1000;
    *(hw->sprint_start_force) = expected_force;

    out = out + leak_fix*20;


    if(out > limit){
        out = limit;
    }
    else if(out < -limit){
        out = -limit;
    }

    out = lowPass->process(out,hw->get_dt());

    last_out = out;

    return out;
}