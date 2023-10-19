#include <forecast/controllers/ForcePID_VC.hpp>

using namespace forecast;

ForcePIDVC::ForcePIDVC(float kp, float ki, float kd, float ps)
    : kp(kp),
      ki(ki),
      kd(kd),
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
      Kqu(0.0f),
      Kd(0.0f)
{
    logs.push_back(&reference);
    lowPass = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    lowPassD = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    lowPassx = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    lowPassDx = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    Ps = 1.00E+5 * ps; // [bar] -> [Pa]
    Be = 1.3E+9f; // Bulk modulus [Pa]
    De = 0.019f;  // Piston diameter [m]
    Dh = 0.0095f; //  Rod diameter [m]
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

    Kd = Ap*((Be/Va0) + pow(alfa,2)*(Be/Vb0));
    Kqu = (Be/Va0)*Kqua + alfa*(Be/Vb0)*Kqub;

    out = ref[0] + kp * err + kd * derr + ki * ierr + Kd * dx / Kqu;

    return out;
}