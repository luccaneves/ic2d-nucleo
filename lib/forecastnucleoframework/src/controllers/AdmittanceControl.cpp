#include <forecast/controllers/AdmittanceControl.hpp>

using namespace forecast;

AdmittanceControl::AdmittanceControl()
    : theta(0.f), err(0.f)
{
    lowPassDTheta = utility::AnalogFilter::getDifferentiatorHz(100.0f);
    lowPassDDTheta = utility::AnalogFilter::getDifferentiatorHz(10.0f);
}

AdmittanceControl::AdmittanceControl(float kp,
                                    float kd,
                                    float ki,
                                    float k_des,
                                    float b_des,
                                    float j_des,
                                    float dob_gain)
    : kp(kp), kd(kd),ki(ki), k_des(k_des), b_des(b_des), j_des(j_des), theta(0.f),
      err(0.f),dob_gain(dob_gain)
{
    /*
        available logs for the user 
    */ 
    logs.push_back(&reference);
    logs.push_back(&tau_err);
    logs.push_back(&theta_ref);
    logs.push_back(&err);
    logs.push_back(&derr);
    logs.push_back(&dtheta_filt);
    logs.push_back(&ddtheta_filt);
    
    lowPassDTheta = utility::AnalogFilter::getDifferentiatorHz(100.0f);
    lowPassDDTheta = utility::AnalogFilter::getDifferentiatorHz(10.0f);
    lowPassExit = utility::AnalogFilter::getLowPassFilterHz(20.0f);

    if(j_des > 0){
        double a_ADM[3] = {j_des,b_des,k_des};
        double b_ADM[3] = {0.0,0.0,1.0};   
        admittanceTF = new utility::AnalogFilter(2, a_ADM, b_ADM);
    }

    else {
        double a_ADM[2] = {b_des,k_des};
        double b_ADM[2] = {0.0,1.0};   
        admittanceTF = new utility::AnalogFilter(1, a_ADM, b_ADM); 
    }
}



float AdmittanceControl::process(const IHardware *hw, std::vector<float> ref)
{

    double inv_model_num[6] = { 0   ,1.592230835850342 , -1.582310443952793 ,0,0 , 0};

    double inv_model_den[6] = {1.000000000000000  ,-1.984087638487695  , 0.984127320055285, 0   ,0,0};


    double filter_num[6] = {0  , 0.310425427160331E-4  , 0.308362809159582E-4 ,0,0,0};
    double filter_den[6] = {1.000000000000000  ,-1.980136794483123 ,  0.980198673306755,0,0,0};
        //double filter_den[6] = {0, 0,0,0,0,0};

    double inv_model_exit = 0;

    double filter_exit = 0;

    tau = hw->get_tau_s(1);
    dtau = hw->get_d_tau_s(1);
    theta = hw->get_theta(0);
    dtheta = hw->get_d_theta(0);
    ddtheta = hw->get_dd_theta(0);

    reference = ref[0];

    dtheta_filt = lowPassDTheta->process(theta, hw->get_dt());
    ddtheta_filt = lowPassDDTheta->process(dtheta, hw->get_dt());

    /* FORCE LOOP */
    tau_err = (tau);
    //Lucca: n tenho certeza disso...
    
    theta_ref = admittanceTF->process(tau_err,hw->get_dt());

    /* POSITION LOOP */
    err_adm = (ref[0] + theta_ref) - theta;
    
    derr_adm = (2.45*err_adm - 6*last_erro_1 + 7.5*last_erro_2 - 6.66*last_erro_3 + 3.75*last_erro_4 - 1.2*last_erro_5 + 0.16*last_erro_6)/(hw->get_dt());

    //derr_adm = lowPass->process(derr_adm,hw->get_dt());

    ierr_adm += err_adm*hw->get_dt();
    
    last_erro_6 = last_erro_5;
    last_erro_5 = last_erro_4;
    last_erro_4 = last_erro_3;
    last_erro_3 = last_erro_2;
    last_erro_2 = last_erro_1;
    last_erro_1 = err_adm;

    if(hw->get_current_time() > 0){
        float value = hw->get_tau_s(1);
        value = dtheta;

        inv_model_exit = 
        inv_model_num[0]*value + 
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
        filter_num[0]*(*(hw->fric2)) + 
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

        filter_exit = filter_exit/filter_den[0];



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
        controller_prev1_tauM = *(hw->fric2);

        controller_prev6_tauSensor = controller_prev5_tauSensor;
        controller_prev5_tauSensor = controller_prev4_tauSensor;
        controller_prev4_tauSensor = controller_prev3_tauSensor;
        controller_prev3_tauSensor = controller_prev2_tauSensor;
        controller_prev2_tauSensor = controller_prev1_tauSensor;
        controller_prev1_tauSensor = value;
    }

    
    out = kp*err_adm + kd*derr_adm + ki*ierr_adm;

    int a = 30;

    double dob_exit = (tau + inv_model_exit) - filter_exit;

    if(dob_exit > a){
        dob_exit = a;
    }

    if(dob_exit < -a){
        dob_exit = -a;
    }
    
    *(hw->var1) = out;
    *(hw->var2) = ref[0];
    *(hw->var3) = dtheta;
    *(hw->var4) = dob_exit;

    out -= dob_gain*dob_exit;

    out = lowPassExit->process(out,hw->get_dt());

    *(hw->fric2) = out;


    return out;  
}

float AdmittanceControl::process_mtx(const IHardware *hw, std::vector<float> ref)
{
    using namespace Eigen;

    // Def.:
    //        b0 s^3 + b1 s^2 + b2 s + b3
    // FT = --------------------------------, with a0 = 1!
    //        a0 s^3 + a1 s^2 + a2 s + a3
    // Forma Canonica Controlavel | (Ogata pg. 596):
    Matrix3f A; 
    A << 0, 1, 0, 0, 0, 1, -3, -2, -1;
    Vector3f B(0, 0, 1);
    RowVector3f C(3 - 3*0, 2 - 2*1, 1 - 1*2);
    float D = 0.2;
    // Discretizacao 2 Ord:
    float Ts = hw->get_dt();
    Matrix3f At_square = (A*Ts) * (A*Ts);
    Matrix3f Ak = Matrix3f::Identity() + A*Ts + At_square/2;
    Vector3f Bk = (Matrix3f::Identity() + A*Ts/2 + At_square/6)*B*Ts;

    tau_err = ref[0] - tau;
    Vector3f xk(tau_err, 0, 0);
    xk = Ak*xk + Bk*tau_err;
    float yk = C*xk + D*tau_err;

    return yk;
}