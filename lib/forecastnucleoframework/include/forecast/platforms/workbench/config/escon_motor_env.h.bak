#ifndef CONFIG_ENV_ESCON_MOTOR_H
#define CONFIG_ENV_ESCON_MOTOR_H

namespace envMotor{

    constexpr PinName MOTOR_ENABLE_PIN = PC_11;
    constexpr PinName MOTOR_CURRENT_FEEDBACK_PIN = PA_1;
    constexpr PinName MOTOR_ANALOG_PIN = PA_5;


    constexpr float MAX_CURR = 3.30f; // is it actually the max analog voltage level?
    
    constexpr float LINMOT_FORCE_MAX = 255.00;
    constexpr float LINMOT_FORCE_MIN = -255.00;
    constexpr float KT = (LINMOT_FORCE_MAX - LINMOT_FORCE_MIN);
    
    //constexpr float HYD_CURRENT_MAX = 10.00;
    //constexpr float HYD_CURRENT_MIN = -10.00;
    //constexpr float KT = (HYD_CURRENT_MAX - HYD_CURRENT_MIN);
    
    constexpr float JM = 1.0000f;
    constexpr float offset_bias = -0.0; // LinMot: 0.000 | Moog E024: -0.0195
    constexpr float amp_scale = 1.00;
}

#endif // CONFIG_CONTROL_ESCON_MOTOR_H