#include <forecast/reference_generators/Sinusoid.hpp>

forecast::SinusoidRefGen::SinusoidRefGen(float frequency, float amplitude) : frequency(frequency), amplitude(amplitude) {
    // ntd;
}

std::vector<float> forecast::SinusoidRefGen::process(const IHardware* hw) {
float output = (amplitude * (-1)* cos(2 * M_PI * frequency * hw->get_current_time() )) + amplitude;

float time = hw->get_current_time();
float duration = hw->get_duration_time();

if (time < duration - 2.0f){
        return {output};
    } 
else {
        return {0.0};
    }
    //return {output};
}
