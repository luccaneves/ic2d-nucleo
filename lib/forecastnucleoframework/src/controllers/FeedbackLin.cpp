#include <forecast/controllers/FeedbackLin.hpp>

using namespace forecast;

FeedbackLin::FeedbackLin(float kp,float kd,float ki,float Kvc,float Kpc)
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
      Pa(0.0f),
      Pb(0.0f),
      f(0.0f),
      g(0.0f),
      v(0.0f)
      
{
    lowPass = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    lowPassD = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    lowPassx = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    lowPassDx = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    lowPassPa = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    lowPassPb = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    
    Be = 1.3E+9f; // Bulk modulus [Pa]
    De = 0.019f;  // Piston diameter [m]
    Dh = 0.0095f; //  Rod diameter [m]
    L_cyl = 0.08f; // Stroke [m]
    Vpl = 1.21E-3f; // Volume Pipeline [m^3]
    In = 0.01f; //  Nominal valve input for Moog 24 [A]
    pn = 70.0E+5f; // Nominal pressure drop for Moog 24 [Pa]
    qn = 0.000125f; // Nominal flow for Moog 24 [m^3/s]
    Fv = 250.0f; // Frequency for Moog 24 [Hz]
    

    logs.push_back(&reference);
    
}

float FeedbackLin::process(const IHardware *hw, std::vector<float> ref)
{

    reference = ref[0];
    
    tau = lowPass->process(hw->get_tau_s(1), hw->get_dt());
    dtau = lowPassD->process(hw->get_d_tau_s(1), hw->get_dt());

    x = lowPassx->process(hw->get_theta(0), hw->get_dt());
    dx = lowPassDx->process(hw->get_d_theta(0), hw->get_dt());

    Pa = lowPassPa->process(hw->get_pressure(0), hw->get_dt());
    Pb = lowPassPb->process(hw->get_pressure(1), hw->get_dt());
    //Ps = lowPassPs->process(hw->get_pressure(2), hw->get_dt());
    //Pt = lowPassPt->process(hw->get_pressure(3), hw->get_dt());

    ixv = hw->get_tau_m(0);

    err = ref[0] - tau;
    derr = (err - errPast) / hw->get_dt();
    ierr += err * hw->get_dt();
    errPast = err;

    float De2 = pow(De, 2);
    float Dh2 = pow(Dh, 2);

    Aa = (M_PI*(De2))/4;
    Ab = ((M_PI*(De2))/4) - ((M_PI*(Dh2))/4);
    Ap = Aa;                    
    alfa = Ab/Aa;
    Kv = qn/sqrt(pn);
    
    Va = Vpl + Aa*x;
    Vb = Vpl + (L_cyl - x)*Ab;

    if(ixv >= 0.0f){
        g = Be*Aa*Kv*( round((Ps-Pa)/abs(Ps-Pa))*sqrt(abs(Ps-Pa))/Va + alfa*round((Pb-Pt)/abs(Pb-Pt))*sqrt(abs(Pb-Pt))/Vb );}
    else{
        g = Be*Aa*Kv*( round((Pa-Pt)/abs(Pa-Pt))*sqrt(abs(Pa-Pt))/Va + alfa*round((Ps-Pb)/abs(Ps-Pb))*sqrt(abs(Ps-Pb))/Vb );}

    f = Be*pow(Aa,2)*( -pow(alfa,2)/Vb - 1/Va )*dx;

    v = ref[0] + kp * err + kd * derr + ki * ierr;

    out = Kpc/(g)*(-Kvc*f + v);

    return out;
}