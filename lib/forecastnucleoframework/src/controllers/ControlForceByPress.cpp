#include <forecast/controllers/ControlForceByPress.hpp>

using namespace forecast;


ControlForceByPress::ControlForceByPress(float kp_force, float ki_force, float kd_force,
float kp_press, float ki_press, float kd_press)
    : kp_press(kp_press), 
    ki_press(ki_press),
    kd_press(kd_press),
    kp_force(kp_force), 
    ki_force(ki_force),
    kd_force(kd_force)
{

    lowPass = utility::AnalogFilter::getLowPassFilterHz(20.0f);
    lowPassPa = utility::AnalogFilter::getLowPassFilterHz(5.0f);
    lowPassPb = utility::AnalogFilter::getLowPassFilterHz(5.0f);
    lowPassOut = utility::AnalogFilter::getLowPassFilterHz(10.0f);
        
    for(int i =0; i < 7; i++){
        f[i] = 0;
    }
    
}

float ControlForceByPress::process(const IHardware *hw, std::vector<float> ref)
{
    dx = hw->get_d_theta(1);
    tau = hw->get_tau_s(1);
    reference = ref[0];

    err_tau = ref[0] - tau; 
    derr_tau = (err_tau - errPast_tau) / hw->get_dt();
    ierr_tau += err_tau * hw->get_dt();

    errPast_tau = err_tau;

    out_tau = kp_force * err_tau + ki_force * ierr_tau + kd_force * derr_tau + ref[0] + 700*dx;

    float h = hw->get_dt();
    int i = 6; 

    /*df_x = (-2.8813923412198247*f[i-15]+4.629184832680677e+1*f[i-14]-3.488506853855236e+2*f[i-13]
        +1.6371494170793765e+3*f[i-12]-5.356573866755776e+3*f[i-11]+
        1.2960925387254853e+4*f[i-10]-2.4002371835256935e+4*f[i-9]+
        3.47295763681826e+4*f[i-8]-3.972640897740953e+4*f[i-7]+3.611601684809462e+4*f[i-6]-
        2.6100690653443193e+4*f[i-5]+1.4939116320184834e+4*f[i-4]-
        6.738488415149466e+3*f[i-3]+2.405588145952734e+3*f[i-2]-7.35230823608962e+2*f[i-1]+
        1.7629026675715002e+2*f[i+0])/(5.655601165399195e+1*1.0*h);*/


    /*df_x = (10*f[i-6]-72*f[i-5]+225*f[i-4]-400*f[i-3]+450*f[i-2]-360*f[i-1]+147*f[i+0])/(60*1.0*h*1);

    f[0] = f[1];
    f[1] = f[2];
    f[2] = f[3];
    f[3] = f[4];
    f[4] = f[5];
    f[5] = f[6];
    f[6] = ref[0] - tau;*/

   /* f[7] = f[8];
    f[8] = f[9];
    f[9] = f[15];
    f[10] = f[11];
    f[11] = f[12];
    f[12] = f[13];
    f[13] = f[14];
    f[14] = f[15];
    f[15] = ref[0] - tau;*/


    float out_tau_filter = (last_out_1*0.016528546178383 + 0.983471453821617*last_out_filter_1);

    last_out_1 = out_tau;
    last_out_filter_1 = out_tau_filter;

    Pa =  lowPassPa->process(hw->get_pressure(2)*100000,hw->get_dt());
    Pb = lowPassPb->process(hw->get_pressure(3)*100000,hw->get_dt());
    //reference_pl = ref[0];

    Pl=Pa-0.609375*Pb;

    err = (out_tau_filter)/(0.00020106) - Pl;
    derr = (err - errPast)/ hw->get_dt();
    ierr += err* hw->get_dt();

    errPast = err;

    out = (kp_press/1000000)*err + (ki_press/1000000)*ierr + (kd_press/1000000)*derr;

    //out = lowPassOut->process(out,hw->get_dt());

    if(hw->get_current_time() > 5 && once_rise_time_flag == 0 && tau > ref[0]*0.1){
            once_rise_time_flag = 1;
            rise_time_start = hw->get_current_time();
    }

    else if(once_2_rise_time_flag == 0 && once_rise_time_flag == 1 && hw->get_current_time() > 5 && tau > ref[0]*0.9){
            rise_time_end = hw->get_current_time();
            once_2_rise_time_flag = 1;
    }

    if(tau > Mv && hw->get_current_time() > 5){
        Mv = tau;
    }

    *(hw->var1) = out_tau_filter;
    *(hw->var2) = rise_time_end - rise_time_start;
    *(hw->var6) = Mv;
    *(hw->var7) = Pl;
    *(hw->var8) = out_tau_filter/(0.00020106); //Pl desejado
    *(hw->var9) = ref[0]; //Forca desejada
    

    return out;

}