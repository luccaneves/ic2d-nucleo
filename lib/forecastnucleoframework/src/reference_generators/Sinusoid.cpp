#include <forecast/reference_generators/Sinusoid.hpp>

forecast::SinusoidRefGen::SinusoidRefGen(float frequency, float amplitude) : frequency(frequency), amplitude(amplitude) {
    // ntd;
}

std::vector<float> forecast::SinusoidRefGen::process(const IHardware* hw) {
//float output = (amplitude * (-1)* cos(2 * M_PI * frequency * hw->get_current_time() )) + amplitude;
float output = (amplitude * sin(2 * M_PI * frequency * hw->get_current_time()));

float time = hw->get_current_time();
float duration = hw->get_duration_time();

//std::vector<float> return_value = {0,0,0};
//return_value[1] = amplitude;
//return_value[2] = frequency;

if (time > 5.0f && time < duration - 5.0f){
        //return_value[0] = output;
        //return_value[1] = amplitude;
        //return_value[2] = frequency;
        //return {return_value};
        return {output};
    } 
else {
        return {0.0};
    }
    //return {output};
}
