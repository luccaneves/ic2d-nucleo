#include <forecast/controllers/ForcePID_VC.hpp>

using namespace forecast;

ForcePIDVC::ForcePIDVC(float kp, float ki, float kd, float Kvc)
    : kp(kp),
      ki(ki),
      kd(kd),
      Kvc(Kvc),
      errPast(0.f),
      err(0.f),
      derr(0.f),
      ierr(0.f),
      tau(0.0f),
      dtau(0.0f),
      x(0.f),
      dx(0.f),
      Aa(0.0f),
      Ap(0.0f),
      Ab(0.0f),
      alfa(0.0f),
      Va(0.0f),
      Vb(0.0f),
      Va0(0.0f),
      Vb0(0.0f),
      Wv(0.0f),
      Kv(0.0f),
      Kvp(0.0f),
      Kqua(0.0f),
      Kqub(0.0f),
      Pa0(0.0f),
      Pb0(0.0f),
      Pt(0.0f),
      Ps(0.0f),
      Kqu(0.0f),
      Kd(0.0f)
{
    logs.push_back(&reference);
    lowPass = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    lowPassD = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    lowPassx = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    lowPassDx = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    lowPassPs = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    lowPassPt = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    //Ps = 1.00E+5 * ps; // [bar] -> [Pa]
    Be = 1.3E+9f; // Bulk modulus [Pa]
    De = 0.016f;  // Piston diameter [m]
    Dh = 0.010f; //  Rod diameter [m]
    L_cyl = 0.08f; // Stroke [m]
    Vpl = 1.21E-3f; // Volume Pipeline [m^3]
    In = 0.01f; //  Nominal valve input for Moog 24 [A]
    pn = 70.0E+5f; // Nominal pressure drop for Moog 24 [Pa]
    qn = 0.000125f; // Nominal flow for Moog 24 [m^3/s]
    Fv = 250.0f; // Frequency for Moog 24 [Hz]
    Dv = 0.5f; // Mistério
    
}

float ForcePIDVC::process(const IHardware *hw, std::vector<float> ref)
{
    reference = ref[0];
    //tau = hw->get_tau_s(1);     // was 0: tauS
    //dtau = hw->get_d_tau_s(1);  // was 0: tauS

    tau = lowPass->process(hw->get_tau_s(1), hw->get_dt());
    dtau = lowPassD->process(hw->get_d_tau_s(1), hw->get_dt());

    x = lowPassx->process(hw->get_theta(0), hw->get_dt());
    dx = lowPassDx->process(hw->get_d_theta(0), hw->get_dt());

    Ps = lowPassPs->process(hw->get_pressure(2), hw->get_dt());
    Pt = lowPassPt->process(hw->get_pressure(3), hw->get_dt());

    err = ref[0] - tau;
    derr = (err - errPast) / hw->get_dt();
    ierr += err * hw->get_dt();
    errPast = err;

    float De2 = pow(De, 2);
    float Dh2 = pow(Dh, 2);
    float B = 1000;
    float ddx = hw->get_dd_theta();

    Aa = (M_PI*(De2))/4;
    Ab = ((M_PI*(De2))/4) - ((M_PI*(Dh2))/4);
    Ap = Aa;                    
    alfa = Ab/Aa;
    Va = Aa*(L_cyl/2);
    Vb = Ab*(L_cyl/2);
    Va0 = Vpl + Va;
    Vb0 = Vpl + Vb;
    Wv = 2*M_PI*Fv;
    Kv = qn/sqrt(pn);
    Kvp = sqrt(2)*Kv;
    Pa0 = Ps/2;               
    Pb0 = Ps/2; 

    Kqua = (Kvp/In)*sqrt(Ps-Pa0);
    Kqub = (Kvp/In)*sqrt(Pb0-Pt);

    Kd = -Ap*Ap*((Be/Va0) + pow(alfa,2)*(Be/Vb0));
    Kqu = Ap*((Be/Va0)*Kqua + alfa*(Be/Vb0)*Kqub);

    *(hw->vel_comp_value) = Kd * dx / Kqu;

    out = /*ref[0] +*/ kp * err + kd * derr + ki * ierr + Kvc*( -Kd * dx / Kqu + B*ddx/Kqu);

    return out;
}