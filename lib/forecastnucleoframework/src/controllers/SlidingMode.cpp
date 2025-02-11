#include <forecast/controllers/SlidingMode.hpp>

using namespace forecast;

SlidingMode::SlidingMode(float max_f, float min_f, float max_g, float min_g, float eta, float psi, float limit, float gain_out, float gain_dob, float limit_dob, float lambda
,float max_disturb_current, float min_disturb_current, float disturb_model_gain, float kp, float ki, float kd, 
float sensor_select, float start_x, float freq)
    : 
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
      limit(limit),
      gain_out(gain_out),
      max_f(max_f),
      max_g(max_g),
      min_f(min_f),
      min_g(min_g),
      psi(psi),
      etta(eta),
      gain_dob(gain_dob),
      limit_dob(limit_dob),
      lambda(lambda),
      max_disturb_current(max_disturb_current),
      min_disturb_current(min_disturb_current),
      disturb_model_gain(disturb_model_gain),
      kp(kp),
      ki(ki),
      kd(kd),
      sensor_select(sensor_select),
      start_x(start_x),
      freq(freq)

{
    float freq_filtro = freq;
    lowPass = utility::AnalogFilter::getLowPassFilterHz(freq_filtro);
    lowPassD = utility::AnalogFilter::getLowPassFilterHz(freq_filtro);
    lowPassx = utility::AnalogFilter::getLowPassFilterHz(freq_filtro);
    lowPassDx = utility::AnalogFilter::getLowPassFilterHz(freq_filtro);
    lowPassPa = utility::AnalogFilter::getLowPassFilterHz(freq_filtro);
    lowPassPb = utility::AnalogFilter::getLowPassFilterHz(freq_filtro);
    lowPassPs = utility::AnalogFilter::getLowPassFilterHz(freq_filtro);
    lowPassPt = utility::AnalogFilter::getLowPassFilterHz(freq_filtro);
    
    Be = 1.31E+9f; // Bulk modulus [Pa]
    De = 0.016f;  // Piston diameter [m]
    Dh = 0.01f; //  Rod diameter [m]
    L_cyl = 0.08f; // Stroke [m]
    //L_cyl = 0.32f; // Stroke [m]
    Vpl = 0.00121; // Volume Pipeline [m^3]
    In = 0.05f; //  Nominal valve input for Moog 24 [A]
    pn = 70.0E+5f; // Nominal pressure drop for Moog 24 [Pa]
    qn = 0.000166666f; // Nominal flow for Moog 24 [m^3/s]
    

    logs.push_back(&reference);
    
}

float SlidingMode::process(const IHardware *hw, std::vector<float> ref)
{
    float start_time = 0;
    uint32_t force_sensor_number = sensor_select;


    //Kvc = Kvc*0.089;
    //Kpc = Kpc*0.089;
    reference = ref[0];
    
    tau = lowPass->process(hw->get_tau_s(force_sensor_number),hw->get_dt());
    dtau = lowPassD->process(hw->get_d_tau_s(force_sensor_number),hw->get_dt());
    float tau_no_filt = hw->get_tau_s(force_sensor_number);

    x = hw->get_theta(1);
    dx = hw->get_d_theta(1);

    if (once == 1 && hw->get_current_time() > start_time/2)
    {
        once_force = hw->get_tau_s(force_sensor_number);
        offset_x = x;
        once = 0;
    }

    if(hw->get_current_time() > start_time){

        Pa = lowPassPa->process(hw->get_pressure(2)*100000,hw->get_dt());
        Pb = lowPassPb->process(hw->get_pressure(3)*100000,hw->get_dt());
        Ps = 16000000;
        Pt = 0;
        Pt = 0; // Sensor de pressão com problema

        if(Pa == Ps){
            Pa = Ps*0.99;
        }

        if(Pb == Ps){
            Pb = Ps*0.99;
        }

        ixv = last_out - 0.0250*0;
        //ixv = last_out;
        //Corrigir a leitura da corrente para checar qual equação de g utilizar

        err = ref[0] - tau;
        derr = (err - errPast) / hw->get_dt();

        prev_erro_7 = prev_erro_6;
        prev_erro_6 = prev_erro_5;
        prev_erro_5 = prev_erro_4;
        prev_erro_4 = prev_erro_3;
        prev_erro_3 = prev_erro_2;
        prev_erro_2 = prev_erro_1;
        prev_erro_1 = err;

        float dist_gain_max = max_g*max_disturb_current;
        float dist_gain_min = min_g*min_disturb_current;

        float dist_gain_med = dist_gain_max/2 + dist_gain_min/2;



        ierr += err * hw->get_dt();
        errPast = err;

        float De2 = pow(De, 2);
        float Dh2 = pow(Dh, 2);

        Aa = (M_PI*(De2))/4;
        Ab = ((M_PI*(De2))/4) - ((M_PI*(Dh2))/4);
        Ap = Aa;                    
        alfa = Ab/Aa;
        Kv = qn/(In*sqrt(pn/2));

        Va = Vpl + Aa*(x + start_x);
        Vb = Vpl + (L_cyl - (x + start_x))*Ab;

        if(ixv >= 0.00000f){
            g = Be*Aa*Kv*(round((Ps-Pa)/abs(Ps-Pa))*sqrt(abs(Ps-Pa))/Va + alfa*round((Pb-Pt)/abs(Pb-Pt))*sqrt(abs(Pb-Pt))/Vb);
            }
            
        else{
            g = Be*Aa*Kv*(round((Pa-Pt)/abs(Pa-Pt))*sqrt(abs(Pa-Pt))/Va + alfa*round((Ps-Pb)/abs(Ps-Pb))*sqrt(abs(Ps-Pb))/Vb);
            }

        //g = Be*Aa*Kv*( round((Pa-Pt)/abs(Pa-Pt))*sqrt(abs(Pa-Pt))/Va + alfa*round((Ps-Pb)/abs(Ps-Pb))*sqrt(abs(Ps-Pb))/Vb );


        f = -Be*pow(Aa,2)*(pow(alfa,2)/Vb + 1/Va)*dx;

        beta = sqrt(max_g/min_g);
        gain_g_med = sqrt(max_g*min_g);
        gain_f_med = (max_f + min_f)/2;

        dtau_desejada = (2.45*ref[0] - 6*prev_ref_1 + 7.5*prev_ref_2 - 6.66*prev_ref_3 
        + 3.75*prev_ref_4 - 1.2*prev_ref_5 + 0.16*prev_ref_6)/
        (hw->get_dt());


        dtau_desejada = (reference - prev_ref_1)/ (hw->get_dt());

        prev_ref_6 = prev_ref_5;
        prev_ref_5 = prev_ref_4;
        prev_ref_4 = prev_ref_3;
        prev_ref_3 = prev_ref_2;
        prev_ref_2 = prev_ref_1;
        prev_ref_1 = ref[0];

        float u = (dtau_desejada - gain_f_med*f + disturb_model_gain*dist_gain_med*g);

        float k = (beta*(abs((max_f - gain_f_med)*(f)) + etta) + abs(beta - 1)*abs(u) + beta*(abs(dist_gain_max - dist_gain_med)*(g)) + kp*(ref[0] - tau) + ki*ierr + kd*derr);

        float sat_ = 0;
        float s = tau - ref[0];

        if(abs(s/psi) <= 1){
            sat_ = s/psi;
            }
        else{
            if(s >= 0){
            sat_ = 1; 
            }
            else if(s < 0){
                sat_ = -1; 
            }
        }
        

        //current = 1/(0.86*g)*(u - k*sign(s));
        out = ((u + kp*(ref[0] - tau) + ki*ierr + kd*derr - k*sat_)*1000)/(gain_g_med*g);

        if(out > limit){
            out = limit;
        }
        else if(out < -limit){
            out = -limit;
        }

        last_out = out;

        *(hw->var1) = out;
        *(hw->var2) = tau - ref[0];
        *(hw->var3) = ref[0];
        *(hw->var4) = tau;
        *(hw->var5) = sat_;
    }
    else{
        out = 0;
    }


    if(hw->get_current_time() > 5 && once_rise_time_flag == 0 && tau_no_filt > ref[0]*0.1){
            once_rise_time_flag = 1;
            rise_time_start = hw->get_current_time();
    }

    else if(once_2_rise_time_flag == 0 && once_rise_time_flag == 1 && hw->get_current_time() > 5 && tau_no_filt > ref[0]*0.9){
            rise_time_end = hw->get_current_time();
            once_2_rise_time_flag = 1;
    }

    *(hw->var6) = Mv;
    *(hw->var8) = -rise_time_start + rise_time_end;

    //*(hw->var4) = out;

    //Lucca: Adicionado filtro na saída. Vai dar merda?


    if(tau > Mv && hw->get_current_time() > 5){
        Mv = tau;
    }
    //*(hw->var4) = out;
    //out = lowPass->process(out,hw->get_dt());



    return out*gain_out;
}