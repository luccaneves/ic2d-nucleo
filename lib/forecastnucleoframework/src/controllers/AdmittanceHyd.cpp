#include <forecast/controllers/AdmittanceHyd.hpp>

using namespace forecast;

AdmittanceHyd::AdmittanceHyd(float kp,float kd,float ki, float Kdes, float Bdes,float Mdes)
    : kp(kp),
      kd(kd),
      ki(ki),
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
      Kdes(Kdes),
      Bdes(Bdes),
      Mdes(Mdes)
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
    qn = 0.0001666f; // Nominal flow for Moog 24 [m^3/s]
    

    logs.push_back(&reference);

    if(Mdes > 0){
        double a_ADM[3] = {Mdes,Bdes,Kdes};
        double b_ADM[3] = {0.0,0.0,1.0};   
        admittanceTF = new utility::AnalogFilter(2, a_ADM, b_ADM);
    }

    else {
        double a_ADM[2] = {Bdes,Kdes};
        double b_ADM[2] = {0.0,1.0};   
        admittanceTF = new utility::AnalogFilter(1, a_ADM, b_ADM); 
    }
    
}

float AdmittanceHyd::PositionController(const IHardware *hw, float ref){
    err_adm = ref;
    derr_adm = (2.45*err_adm - 6*last_erro_1 + 7.5*last_erro_2 - 6.66*last_erro_3 + 3.75*last_erro_4 - 1.2*last_erro_5 + 0.16*last_erro_6)/(hw->get_dt());

    ierr_adm += err_adm*hw->get_dt();

    last_erro_6 = last_erro_5;
    last_erro_5 = last_erro_4;
    last_erro_4 = last_erro_3;
    last_erro_3 = last_erro_2;
    last_erro_2 = last_erro_1;
    last_erro_1 = err_adm;

    float out = kp*err_adm + kd*derr_adm + ki*ierr_adm;

    return out;

}

float AdmittanceHyd::process(const IHardware *hw, std::vector<float> ref)
{

    //Kvc = Kvc*0.089;
    //Kpc = Kpc*0.089;
    reference = ref[0];
    
    tau = hw->get_tau_s(0);
    dtau = hw->get_d_tau_s(0);

    x = hw->get_theta(1);
    dx = hw->get_d_theta(1);
    ddx = hw->get_dd_theta(1);

    if(once == 1 && hw->get_current_time() > 1){
        once = 0;
        start_force = tau;
    }

    deriv_ref = (reference - prev1_ref_x_1)/(hw->get_dt());

    prev1_ref_x_6 = prev1_ref_x_5;
    prev1_ref_x_5 = prev1_ref_x_4;
    prev1_ref_x_4 = prev1_ref_x_3;
    prev1_ref_x_3 = prev1_ref_x_2;
    prev1_ref_x_2 = prev1_ref_x_1;
    prev1_ref_x_1 = reference;

    theta_ref = admittanceTF->process((tau - start_force),hw->get_dt());

    float ref_adm = (ref[0] + theta_ref) - x;

    float out;

    if(hw->get_current_time() < 1.2){
        out = 0;
    }
    else{
        out = PositionController(hw,ref_adm);
    }


    *(hw->var5) = ref_adm;
    *(hw->var6) = theta_ref;

    *(hw->var7) = deriv_ref;
    *(hw->var8) = tau - start_force;
    *(hw->var9) = reference;

    if(hw->get_current_time() < 1.2){
        out = 0;
    }

    *(hw->var4) = out;

    return out;
}