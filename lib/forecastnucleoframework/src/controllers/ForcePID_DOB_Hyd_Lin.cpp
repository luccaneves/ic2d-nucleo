#include <forecast/controllers/ForcePID_DOB_Hyd_Lin.hpp>

using namespace forecast;

ForcePID_DOB_Hyd_Lin::ForcePID_DOB_Hyd_Lin(float kp, float ki, float kd)
    : kp(kp),
      ki(ki),
      kd(kd),
      errPast(0.f),
      err(0.f),
      derr(0.f),
      ierr(0.f)
{
    logs.push_back(&reference);
    lowPass = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    lowPassD = utility::AnalogFilter::getLowPassFilterHz(40.0f);
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

    lowPass = utility::AnalogFilter::getLowPassFilterHz(10.0f);
    lowPassD = utility::AnalogFilter::getLowPassFilterHz(10.0f);
    lowPassx = utility::AnalogFilter::getLowPassFilterHz(10.0f);
    lowPassDx = utility::AnalogFilter::getLowPassFilterHz(10.0f);
    lowPassPa = utility::AnalogFilter::getLowPassFilterHz(10.0f);
    lowPassPb = utility::AnalogFilter::getLowPassFilterHz(10.0f);
    lowPassPs = utility::AnalogFilter::getLowPassFilterHz(10.0f);
    lowPassPt = utility::AnalogFilter::getLowPassFilterHz(10.0f);
}



float ForcePID_DOB_Hyd_Lin::process(const IHardware *hw, std::vector<float> ref)
{
    double inv_model_exit = 0;

    double filter_exit = 0;

    double inv_model_num[6] = {0.480000000000000E4 , -1.405982901161782E4  , 1.372015813527966E4 , -0.446032912366184E4,   0, 0};
    double inv_model_den[6] = {1.000000000000000 , -2.999497575612927  , 2.998997800567079 , -0.999500124979168,   0, 0};

    double filter_num[6] = {0 ,  0.166645814447765E-7 ,  0.666499860455253E-7   , 0.166604158477415E-7 ,0,0};
    double filter_den[6] = { 1.000000000000000  ,-2.999497575612929   ,2.998997800567081  ,-0.999500124979169,0,0};

    reference = ref[0];
    
    tau = lowPass->process(hw->get_tau_s(1), hw->get_dt());
    dtau = lowPassD->process(hw->get_d_tau_s(1), hw->get_dt());

    float x = lowPassx->process(hw->get_theta(0), hw->get_dt());
    float dx = lowPassDx->process(hw->get_d_theta(0), hw->get_dt());

    Pa = lowPassPa->process(hw->get_pressure(0), hw->get_dt())*100000;
    Pb = lowPassPb->process(hw->get_pressure(1), hw->get_dt())*100000;
    Ps = lowPassPs->process(hw->get_pressure(2), hw->get_dt())*100000;
    Pt = lowPassPt->process(hw->get_pressure(3), hw->get_dt())*100000;

    float ixv = hw->get_tau_m(0);

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

    float g = 0;

    if(ixv >= 0.0f){
        g = Be*Aa*Kv*( round((Ps-Pa)/abs(Ps-Pa))*sqrt(abs(Ps-Pa))/Va + alfa*round((Pb-Pt)/abs(Pb-Pt))*sqrt(abs(Pb-Pt))/Vb );}
    else{
        g = Be*Aa*Kv*( round((Pa-Pt)/abs(Pa-Pt))*sqrt(abs(Pa-Pt))/Va + alfa*round((Ps-Pb)/abs(Ps-Pb))*sqrt(abs(Ps-Pb))/Vb );}

    //f = Be*pow(Aa,2)*( -pow(alfa,2)/Vb - 1/Va )*dx;
    float f = Be*pow(Aa,2)*( pow(alfa,2)/Vb + 1/Va )*dx;

    float v = /*ref[0] +*/ kp * err + kd * derr + ki * ierr;

    out = Kpc/(g)*(-Kvc*f*1000 + v);

    if(hw->get_current_time() > 0){

        inv_model_exit = 
        inv_model_num[0]*x + 
        inv_model_num[1]*controller_prev1_x + 
        inv_model_num[2]*controller_prev2_x +
        inv_model_num[3]*controller_prev3_x +
        inv_model_num[4]*controller_prev4_x +
        inv_model_num[5]*controller_prev5_x 
        -
        inv_model_den[1]*prev1_inv_model_exit - 
        inv_model_den[2]*prev2_inv_model_exit -
        inv_model_den[3]*prev3_inv_model_exit -
        inv_model_den[4]*prev4_inv_model_exit -
        inv_model_den[5]*prev5_inv_model_exit;

        inv_model_exit = inv_model_exit;

        filter_exit = 
        filter_num[0]*(hw->get_tau_m(0))/1000 + 
        filter_num[1]*controller_prev1_tauM + 
        filter_num[2]*controller_prev2_tauM + 
        filter_num[3]*controller_prev3_tauM + 
        filter_num[4]*controller_prev4_tauM  
        -
        filter_den[1]*prev1_filter_exit - 
        filter_den[2]*prev2_filter_exit -
        filter_den[3]*prev3_filter_exit -
        filter_den[4]*prev4_filter_exit -
        filter_den[5]*prev5_filter_exit;

        filter_exit = filter_exit;



        prev5_filter_exit = prev4_filter_exit;
        prev4_filter_exit = prev3_filter_exit;
        prev3_filter_exit = prev2_filter_exit;
        prev2_filter_exit = prev1_filter_exit;
        prev1_filter_exit = filter_exit;

        prev5_inv_model_exit = prev4_inv_model_exit;
        prev4_inv_model_exit = prev3_inv_model_exit;
        prev3_inv_model_exit = prev2_inv_model_exit;
        prev2_inv_model_exit = prev1_inv_model_exit;
        prev1_inv_model_exit = inv_model_exit;

        controller_prev6_tauM = controller_prev5_tauM;
        controller_prev5_tauM = controller_prev4_tauM;
        controller_prev4_tauM = controller_prev3_tauM;
        controller_prev3_tauM = controller_prev2_tauM;
        controller_prev2_tauM = controller_prev1_tauM;
        controller_prev1_tauM = hw->get_tau_m(0)/1000;

        controller_prev6_x = controller_prev5_x;
        controller_prev5_x = controller_prev4_x;
        controller_prev4_x = controller_prev3_x;
        controller_prev3_x = controller_prev2_x;
        controller_prev2_x = controller_prev1_x;
        controller_prev1_x = x;
    }

    double dob_exit = inv_model_exit - filter_exit;

    return out - dob_exit*1000;
}