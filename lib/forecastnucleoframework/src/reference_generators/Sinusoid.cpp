#include <forecast/reference_generators/Sinusoid.hpp>

forecast::SinusoidRefGen::SinusoidRefGen(float frequency, float amplitude) : frequency(frequency), amplitude(amplitude) {
    // ntd;
}

std::vector<float> forecast::SinusoidRefGen::process(const IHardware* hw) {


float time = hw->get_current_time();
float duration = hw->get_duration_time();

if (time > 5.0f && time < duration - 5.0f){
    float output = (amplitude * -cos(2 * M_PI * frequency * (hw->get_current_time() -5.0 ))) + amplitude;
        return {output};
    } 
else {
        return {0.00};
    }
    //return {output};
}
