#include <forecast/controllers/ForcePID_DOB_Hyd.hpp>

using namespace forecast;

ForcePID_DOB_HYD::ForcePID_DOB_HYD(float kp, float ki, float kd)
    : kp(kp),
      ki(ki),
      kd(kd),
      errPast(0.f),
      err(0.f),
      derr(0.f),
      ierr(0.f)
{
    logs.push_back(&reference);
    lowPass = utility::AnalogFilter::getLowPassFilterHz(10.0f);
    lowPassD = utility::AnalogFilter::getLowPassFilterHz(10.0f);
    lowPassPa = utility::AnalogFilter::getLowPassFilterHz(10.0f);
    lowPassPb = utility::AnalogFilter::getLowPassFilterHz(10.0f);
    lowPassPs = utility::AnalogFilter::getLowPassFilterHz(10.0f);
    lowPassPt = utility::AnalogFilter::getLowPassFilterHz(10.0f);

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



float ForcePID_DOB_HYD::process(const IHardware *hw, std::vector<float> ref)
{
    float x = hw->get_theta(0);
    float dx = hw->get_d_theta(0);
    float ddx = hw->get_dd_theta(0);
    

    Pa = lowPassPa->process(hw->get_pressure(0), hw->get_dt())*100000;
    Pb = lowPassPb->process(hw->get_pressure(1), hw->get_dt())*100000;
    Ps = lowPassPs->process(hw->get_pressure(2), hw->get_dt())*100000;
    Pt = lowPassPt->process(hw->get_pressure(3), hw->get_dt())*100000;

    double inv_model_exit = 0;

    double filter_exit = 0;

    reference = ref[0];
    
    float d_disturb = 0;
    float disturb = 0;
    float z = 0;
    float Ml = 7.5;
    float Bl = 150;
    float Kl = 2500;
    float B = 1000;
    float lambda = 50;
    //float dtau = hw->get_d_tau_m

    tau = hw->get_tau_s(1);
    dtau = hw->get_d_tau_s(1);

    err = ref[0] - tau;
    derr = (err - errPast) / hw->get_dt();

    derr = (2.45*err - 6*prev1_err + 7.5*prev2_err - 6.66*prev3_err 
    + 3.75*prev4_err - 1.2*prev5_err + 0.16*prev6_err)/
    (hw->get_dt());

    ierr += err * hw->get_dt();
    errPast = err;

    float De2 = pow(De, 2);
    float Dh2 = pow(Dh, 2);

    float g;
    float ixv = hw->get_tau_m(0);

    Aa = (M_PI*(De2))/4;
    Ab = ((M_PI*(De2))/4) - ((M_PI*(Dh2))/4);
    Ap = Aa;                    
    alfa = Ab/Aa;
    Kv = qn/sqrt(pn);
    
    Va = Vpl + Aa*x;
    Vb = Vpl + (L_cyl - x)*Ab;

    if(ixv >= 0.0f){
        g = Be*Aa*Kv*( round((Ps-Pa)/abs(Ps-Pa))*sqrt(abs(Ps-Pa))/Va + alfa*round((Pb-Pt)/abs(Pb-Pt))*sqrt(abs(Pb-Pt))/Vb );
    }
    else{
        g = Be*Aa*Kv*( round((Pa-Pt)/abs(Pa-Pt))*sqrt(abs(Pa-Pt))/Va + alfa*round((Ps-Pb)/abs(Ps-Pb))*sqrt(abs(Ps-Pb))/Vb );
    }

    f = Be*pow(Aa,2)*( pow(alfa,2)/Vb + 1/Va )*dx;

    out = /*ref[0] +*/ kp * err + kd * derr + ki * ierr;

    prev6_err = prev5_err;
    prev5_err = prev4_err;
    prev4_err = prev3_err;
    prev3_err = prev2_err;
    prev2_err = prev1_err;
    prev1_err = err;

    float dz = -(lambda/Ml)*(z) + (lambda/Ml)*(Bl*ddx + Kl*dx + B*ddx - f - ((g*ixv)/1000) - lambda*ddx);
    float dz_filt = lowPassD->process(dz, hw->get_dt());
    z = z + dz_filt*hw->get_dt();
    disturb = z + lambda*ddx;

    /*d_disturb = (2.45*disturb - 6*prev1_disturb + 7.5*prev2_disturb - 6.66*prev3_disturb 
    + 3.75*prev4_disturb - 1.2*prev5_disturb + 0.16*prev6_disturb)/
    (hw->get_dt());

    double d_disturb_filt = lowPassD->process(d_disturb, hw->get_dt());*/

    prev6_disturb = prev5_disturb;
    prev5_disturb = prev4_disturb;
    prev4_disturb = prev3_disturb;
    prev3_disturb = prev2_disturb;
    prev2_disturb = prev1_disturb;
    prev1_disturb = disturb;

    prev6_z = prev5_z;
    prev5_z = prev4_z;
    prev4_z = prev3_z;
    prev3_z = prev2_z;
    prev2_z = prev1_z;
    prev1_z = z;

    prev1_dx = dx;

    prev1_ddx = ddx;

    prev1_dsensor = dtau;

    float comp = 0;

    comp = disturb/g;

    /*enviar em mA*/
    comp = comp*1000;

    float limit_sat = 0.1;

    if(comp > limit_sat){
        comp = limit_sat;
    }
    if(comp < -limit_sat){
        comp = -limit_sat;
    }

    *(hw->fric1) = dz_filt;
    *(hw->fric2) = comp;

    return (out - comp);
}