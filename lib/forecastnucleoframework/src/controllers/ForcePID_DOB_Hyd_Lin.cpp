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
}



float ForcePID_DOB_Hyd_Lin::process(const IHardware *hw, std::vector<float> ref)
{
    float x = hw->get_theta(0);
    float ixv = hw->get_tau_m(0);

    Pa = lowPassPa->process(hw->get_pressure(0), hw->get_dt());
    Pb = lowPassPb->process(hw->get_pressure(1), hw->get_dt());
    Ps = lowPassPs->process(hw->get_pressure(2), hw->get_dt());
    Pt = lowPassPt->process(hw->get_pressure(3), hw->get_dt());

    reference = ref[0];
    //tau = hw->get_tau_s(1);     // was 0: tauS
    //dtau = hw->get_d_tau_s(1);  // was 0: tauS

    //tau = lowPass->process(hw->get_tau_s(1), hw->get_dt());
    //dtau = lowPassD->process(hw->get_d_tau_s(1), hw->get_dt());

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

    Aa = (M_PI*(De2))/4;
    Ab = ((M_PI*(De2))/4) - ((M_PI*(Dh2))/4);
    Ap = Aa;                    
    alfa = Ab/Aa;

    Va = Aa*(L_cyl/2);
    Vb = Ab*(L_cyl/2);

    //Va = Vpl + Aa*x;
    //Vb = Vpl + (L_cyl - x)*Ab;

    Va0 = Vpl + Va;
    Vb0 = Vpl + Vb;
    Wv = 2*M_PI*Fv;
    Kv = qn/sqrt(pn);
    Kvp = sqrt(2)*Kv;
    Pa0 = (Ps*(alfa))/(1 + alfa);               
    Pb0 = (Ps*(alfa))/(1 + alfa);   

    if(ixv > 0.0f){
        Kqua = (Kvp/In)*sqrt(Ps-Pa0);
        Kqub = (Kvp/In)*sqrt(Pb0-Pt);
    }
    else{
        Kqua = (Kvp/In)*sqrt(Pa0-Pt);
        Kqub = (Kvp/In)*sqrt(Ps-Pb0);
    }

    Kxp = Ap*((Be/Va0) + pow(alfa,2)*(Be/Vb0));
    Kuv = (Be/Va0)*Kqua + alfa*(Be/Vb0)*Kqub;


    double inv_model_num[6] = {
    (54930876000 - 546*Kxp - 13732719*Kfh),
    (38128173*Kfh + 474*Kxp - 262374444000),
    (1084*Kxp - 21695990*Kfh + 501671096000),
    (- 27113798*Kfh - 956*Kxp - 479999864000),
    (35428661*Kfh - 538*Kxp + 229830028000),
    - 11014423*Kfh + 482*Kxp - 44057692000
    };

    double inv_model_den[6] = {3865208523453*Kuv,
    19275128275635*Kuv,
    38449422954930*Kuv,
    -38349583446870*Kuv,
    19125368521665*Kuv,
    -3815288277447*Kuv};

    double filter_num[6] = {0.124968678162927E-7, 0.374906034488781E-7, 0.374906034488781E-7, 0.124968678162927E-7, 0, 0};
    double filter_den[6] = {1.000000000000000, -2.999497550932578, 2.998997751201133, -0.999500100293613, 0, 0};
    //double filter_den[6] = {0, 0,0,0,0,0};

    double inv_model_exit = 0;

    double filter_exit = 0;

    if(hw->get_current_time() > 0){

        inv_model_exit = 
        inv_model_num[0]*hw->get_tau_s(1) + 
        inv_model_num[1]*controller_prev1_tauSensor + 
        inv_model_num[2]*controller_prev2_tauSensor +
        inv_model_num[3]*controller_prev3_tauSensor +
        inv_model_num[4]*controller_prev4_tauSensor +
        inv_model_num[5]*controller_prev5_tauSensor 
        -
        inv_model_den[1]*prev1_inv_model_exit - 
        inv_model_den[2]*prev2_inv_model_exit -
        inv_model_den[3]*prev3_inv_model_exit -
        inv_model_den[4]*prev4_inv_model_exit -
        inv_model_den[5]*prev5_inv_model_exit;

        inv_model_exit = inv_model_exit/inv_model_den[0];

        filter_exit = 
        filter_num[0]*(hw->get_tau_m(0)) + 
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

        filter_exit = filter_exit/inv_model_den[0];



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
        controller_prev1_tauM = hw->get_tau_m(0);

        controller_prev6_tauSensor = controller_prev5_tauSensor;
        controller_prev5_tauSensor = controller_prev4_tauSensor;
        controller_prev4_tauSensor = controller_prev3_tauSensor;
        controller_prev3_tauSensor = controller_prev2_tauSensor;
        controller_prev2_tauSensor = controller_prev1_tauSensor;
        controller_prev1_tauSensor = hw->get_tau_s(1);
    }





    *(hw->fric1) = inv_model_exit;
    *(hw->fric2) = filter_exit;

    out = /*ref[0] +*/ kp * err + kd * derr + ki * ierr;

    double dob_exit = inv_model_exit - filter_exit;

    float limit_sat = 0.1;

    if(dob_exit > limit_sat){
        dob_exit = limit_sat;
    }
    if(dob_exit < -limit_sat){
        dob_exit = -limit_sat;
    }

    prev6_err = prev5_err;
    prev5_err = prev4_err;
    prev4_err = prev3_err;
    prev3_err = prev2_err;
    prev2_err = prev1_err;
    prev1_err = err;

    out = out - dob_exit;

    return (out - dob_exit);
}