#include <forecast/controllers/ForcePID_FricComp.hpp>

using namespace forecast;

ForcePID_FricComp::ForcePID_FricComp(float kp, float ki, float , float Kspring, float m1, float m2)
    : kp(kp),
      ki(ki),
      kd(kd),
      Kspring(Kspring),
      m1(m1),
      m2(m2),
      errPast(0.f),
      err(0.f),
      derr(0.f),
      ierr(0.f)
{
    logs.push_back(&reference);
    lowPass = utility::AnalogFilter::getLowPassFilterHz(40.0f);
    lowPassD = utility::AnalogFilter::getLowPassFilterHz(40.0f);
}

float ForcePID_FricComp::process(const IHardware *hw, std::vector<float> ref)
{
    float Fric_1 = 0;
    float Fric_2 = 0;

    x = hw->get_theta(0);
    dx = hw->get_d_theta(0);
    ddx = hw->get_dd_theta(0);

    x_e = hw->get_theta(1);
    dx_e = hw->get_d_theta(1);
    ddx_e = hw->get_dd_theta(1);

    tau = hw->get_tau_s(1);
    dtau = hw->get_d_tau_s(1);

    reference = ref[0];
    //tau = hw->get_tau_s(1);     // was 0: tauS
    //dtau = hw->get_d_tau_s(1);  // was 0: tauS

    //tau = lowPass->process(hw->get_tau_s(1), hw->get_dt());
    //dtau = lowPassD->process(hw->get_d_tau_s(1), hw->get_dt());




    if(flag_first_cycle == 0 && hw->get_current_time() > 0.7)//Lucca: ISSO É UMA GAMBIARRA NOJENTA. Culpa do filtro...
    {
        
        float Frac_1_start = tau/2;
        float Frac_2_start = Frac_1_start*(m2/m1); 
        
        start_pos_x_e = x_e;
        start_force_spring = tau + (m2/m1)*tau;


        start_force_spring = Frac_1_start + Frac_2_start;


        *(hw->sprint_start_force) = start_force_spring;
        flag_first_cycle = 1;
    } 

    if(flag_first_cycle == 0){//Lucca: ISSO tbm É UMA GAMBIARRA NOJENTA. Tudo culpa do filtro...
        Fric_1 = 0;

        Fric_2 = 0;
    }

    else{
        //TODO: Verificar lógica de direção

                  
        Fric_1 = hw->get_tau_m(0) - m1*ddx - tau;

        Fric_2 = tau - m2*ddx_e - (Kspring*(x_e - start_pos_x_e) + start_force_spring); //TODO: Voltar

        //Fric_1 = -Fric_1 - Fric_2;

        //Fric_1 = (Kspring*(x_e - start_pos_x_e) + start_force_spring);




        //Fric_2 = +m1*ddx - m2*ddx_e;

        //Fric_1 = tau - (Kspring*(x_e - start_pos_x_e) + start_force_spring) - hw->get_tau_m(0) - Fric_2;

        //Fric_2 = m1*ddx - m2*ddx_e;

        

        /*float limit = 30;

        if((Fric_1) > limit){
            (Fric_1) = limit;
        }
        if((Fric_1) < -limit){
            (Fric_1) = -limit;
        }*/
        
        
        //Fric_2 = 0;

        /*if(dx_e > 0.005 || (dx_e < 0.005 && dx_e > -0.005)){
            Fric_1 = LastForce - m1*ddx - tau - last_fric1;

            Fric_2 = tau - m2*ddx_e - Kspring*(x_e - start_pos_x_e) - start_force_sensor; //TODO: Voltar
        }
        else{
            Fric_1 = -(LastForce - m1*ddx - tau - last_fric1);

            Fric_2 = -(tau - m2*ddx_e - Kspring*(x_e - start_pos_x_e) - start_force_sensor); //TODO: Voltar
        }*/

        /*
        if(Fric_1 > 20){
            Fric_1 = 20;
        }

        if(Fric_1 < -20){
            Fric_1 = -20;
        }
        */

        //TODO: Adicionar lógica para verificar direção da força de atrito

        /*if(Fric_2 < 0 && dx_e > 0){
            Fric_2 = -Fric_2;
            //Fric_2 = tau - m2*ddx_e - Kspring*(x_e - start_pos_x_e) - start_force_sensor; //TODO: Testar
        }

        if(Fric_2 > 0 && dx_e < 0){
            Fric_2 = -Fric_2;
            //Fric_2 = tau - m2*ddx_e - Kspring*(x_e - start_pos_x_e) - start_force_sensor; //TODO: Testar
        }*/



        /*if(Fric_2 < 0){
            Fric_2 = -Fric_2;
            //Fric_2 = tau - m2*ddx_e - Kspring*(x_e - start_pos_x_e) - start_force_sensor; //TODO: Testar
        }*/
    }

    err = ref[0] - tau; //TODO: Voltar


    //Lucca TODO: Corrigir essa derivada. Fazer inf dif com mais pontos
    derr = (err - errPast) / hw->get_dt();
    ierr += err * hw->get_dt();
    errPast = err;

    Fric_1 = Fric_1;

    Fric_2 = Fric_2;

    *(hw->fric1) = Fric_1;
    *(hw->fric2) = Fric_2;

    out = ref[0] + kp * err + kd * derr + ki * ierr; 

    *(hw->control_signal_teste) = (Kspring*(x_e - start_pos_x_e) + start_force_spring); //TODO: Alterar

    LastForce = out + Fric_1 + Fric_2;

    last_fric1 = Fric_1;

    last_ref = ref[0];

    float Fric_Total = Fric_1  - Fric_2;

    //Fric_Total = Fric_1;

    float L = 30;

    if(Fric_Total > L){
        Fric_Total = L;
    }
    else if(Fric_Total < -L){
        Fric_Total = -L;

    }

    return out + Fric_Total;
}