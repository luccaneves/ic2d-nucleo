#include <forecast/controllers/ForcePID_DOB_Hyd_Lin.hpp>

using namespace forecast;

ForcePID_DOB_Hyd_Lin::ForcePID_DOB_Hyd_Lin(float kp, float ki, float kd,float kvc, float kpc, float B_int, 
float gain_dob, float limit_dob, float limit, float gain_out)
    : kp(kp),
      ki(ki),
      kd(kd),
      kpc(kpc),
      kvc(kvc),
      errPast(0.f),
      err(0.f),
      derr(0.f),
      ierr(0.f),
      B_int(B_int),
      gain_dob(gain_dob),
      limit_dob(limit_dob),
      limit(limit),
      gain_out(gain_out)
{
    logs.push_back(&reference);

    Be = 1.31E+9f; // Bulk modulus [Pa]
    De = 0.016f;  // Piston diameter [m]
    Dh = 0.01f; //  Rod diameter [m]
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

    double inv_model_num[6] = { 0   ,1.592230835850342 , -1.582310443952793 ,0,0 , 0};

    double inv_model_den[6] = {1.000000000000000  ,-1.984087638487695  , 0.984127320055285, 0   ,0,0};


    double filter_num[6] = {0  , 0.2212  , 0 ,0,0,0};
    double filter_den[6] = {1.000000000000000  , -0.7788 , 0 ,0,0,0};

    reference = ref[0];
    
    tau = (hw->get_tau_s(1));
    dtau = (hw->get_d_tau_s(1));

    float x = hw->get_theta(0);
    float dx = hw->get_d_theta(0);

    Pt = 0;
    Ps = 10000000;

    float ixv = hw->get_tau_m(0);

    err = ref[0] - tau;
    //derr = (err - errPast) / hw->get_dt();
    ierr += err * hw->get_dt();
    //errPast = err;

    derr = (2.45*err - 6*prev1_err + 7.5*prev2_err - 6.66*prev3_err 
    + 3.75*prev4_err - 1.2*prev5_err + 0.16*prev6_err)/
    (hw->get_dt());

    prev6_err = prev5_err;
    prev5_err = prev4_err;
    prev4_err = prev3_err;
    prev3_err = prev2_err;
    prev2_err = prev1_err;
    prev1_err = err;

    float De2 = pow(De, 2);
    float Dh2 = pow(Dh, 2);

    Aa = (M_PI*(De2))/4;
    Ab = ((M_PI*(De2))/4) - ((M_PI*(Dh2))/4);
    Ap = Aa;                    
    alfa = Ab/Aa;
    Kv = qn/(In*sqrt(pn/2));
    
    Va = Vpl + Aa*x;
    Vb = Vpl + (L_cyl - x)*Ab;

    double va0 = Vpl + L_cyl*Ap;
    double vb0 = Vpl + L_cyl*Ap*alfa;
    double uv0 = 0;
    double pa0 = alfa*Ps/(1 + alfa);
    double pb0 = Ps/(1 + alfa);

    double g = 0;

    double Kth = Be*Ap*(1/va0 + alfa/vb0);

    double Kqa_pos = (Kv)*sqrt(Ps-pa0);
    double Kqb_pos= (Kv)*sqrt(pb0-Pt);

    double Kca_pos = Kv*uv0/(2*sqrt(Ps-pa0));
    double Kcb_pos = -Kv*uv0/(2*sqrt(pb0 - Pt));

    double Kce_pos = (1/(1 + alfa*alfa*alfa))*((vb0*Kca_pos - alfa*alfa*alfa*va0*Kcb_pos)/(vb0 + alfa*va0));

    double Kqe_pos = (vb0*Kqb_pos + alfa*va0*Kqb_pos)/(vb0 + alfa*va0);


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

    float comp_value = (output_filter_dF/Kth + Ap*output_filter_dx*kvc)/(Kqe_pos*kpc) - output_filter_current;
    
    comp_value = comp_value*1000;

    float a = limit_dob;

    if(comp_value > a){
        comp_value = a;
    }
    if(comp_value < -a){
        comp_value = -a;
    }

    float d_expected_force = (((ixv + comp_value)/1000)*Kqe_pos*kpc - dx*Ap*kvc)*Kth;

    if(hw->get_current_time() > 4){
       if(once_force == 1){
        once_force = 0;
        expected_force = tau;
       }
       expected_force = expected_force + d_expected_force*hw->get_dt(); 
    }


    *(hw->var1) = expected_force;
    *(hw->var2) = comp_value;

    out = kp * err + kd * derr + ki * ierr - gain_dob*comp_value;

    float limit_sat = limit;

    if(out > limit_sat){
        out = limit_sat;
    }
    else if(out < -limit_sat){
        out = -limit_sat;
    }

    out = lowPass->process(gain_out*out,hw->get_dt());

    return out;
}